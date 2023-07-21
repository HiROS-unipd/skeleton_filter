// ROS dependencies
#include <tf2/LinearMath/Matrix3x3.h>

// Custom external dependencies
#include "rtb/Filter/Designer.h"
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_filter/KinematicStateFilter.h"

hiros::skeletons::KinematicStateFilter::KinematicStateFilter(
    const FilterType& filter_type, const double& cutoff_frequency,
    const double& sample_frequency, const unsigned int& butterworth_order)
    : filter_type_(filter_type),
      cutoff_frequency_(cutoff_frequency),
      sample_frequency_(sample_frequency),
      butterworth_order_(butterworth_order) {
  switch (filter_type_) {
    case FilterType::StateSpace:
      break;

    case FilterType::Butterworth: {
      auto butterworth_tf{rtb::Filter::butter<double>(
          butterworth_order_, cutoff_frequency_, sample_frequency_)};

      for (auto& filter : butterworth_filters_) {
        filter.setTransferFunction(butterworth_tf);
      }
    }

    break;

    default:
      std::cerr << "KinematicStateFilter Error: Unsupported filter type"
                << std::endl;
      break;
  }
};

void hiros::skeletons::KinematicStateFilter::filter(
    hiros::skeletons::types::KinematicState& state, const double& time) {
  auto prev_time{last_time_};
  auto prev_state{last_state_};

  if (!skeletons::utils::isNaN(state.pose.position)) {
    computePosition(state, time);
    computeLinearVelocity(state);
    computeLinearAcceleration(state);
  }

  if (!skeletons::utils::isNaN(state.pose.orientation)) {
    computeOrientation(state, time);
    computeAngularVelocity(state, prev_state, prev_time);
    computeAngularAcceleration(state, prev_state, prev_time);
  }
}

void hiros::skeletons::KinematicStateFilter::computePosition(
    hiros::skeletons::types::KinematicState& state, const double& time) {
  switch (filter_type_) {
    case FilterType::StateSpace:
      state.pose.position.setX(statespace_filters_[0].filter(
          state.pose.position.x(), time, cutoff_frequency_));
      state.pose.position.setY(statespace_filters_[1].filter(
          state.pose.position.y(), time, cutoff_frequency_));
      state.pose.position.setZ(statespace_filters_[2].filter(
          state.pose.position.z(), time, cutoff_frequency_));
      break;

    case FilterType::Butterworth:
      state.pose.position.setX(
          butterworth_filters_[0].filter(state.pose.position.x()));
      state.pose.position.setY(
          butterworth_filters_[1].filter(state.pose.position.y()));
      state.pose.position.setZ(
          butterworth_filters_[2].filter(state.pose.position.z()));
      break;

    default:
      std::cerr << "KinematicStateFilter Error: computePosition()" << std::endl;
      break;
  }
}

void hiros::skeletons::KinematicStateFilter::computeLinearVelocity(
    hiros::skeletons::types::KinematicState& state) {
  switch (filter_type_) {
    case FilterType::StateSpace:
      state.velocity.linear.setX(
          statespace_filters_[0].getFilteredFirstDerivative());
      state.velocity.linear.setY(
          statespace_filters_[1].getFilteredFirstDerivative());
      state.velocity.linear.setZ(
          statespace_filters_[2].getFilteredFirstDerivative());
      break;

    case FilterType::Butterworth:
      // TODO: Not supported yet
      state.velocity.linear = {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::quiet_NaN()};
      break;

    default:
      std::cerr << "KinematicStateFilter Error: computeLinearVelocity()"
                << std::endl;
      break;
  }
}

void hiros::skeletons::KinematicStateFilter::computeLinearAcceleration(
    hiros::skeletons::types::KinematicState& state) {
  switch (filter_type_) {
    case FilterType::StateSpace:
      state.acceleration.linear.setX(
          statespace_filters_[0].getFilteredSecondDerivative());
      state.acceleration.linear.setY(
          statespace_filters_[1].getFilteredSecondDerivative());
      state.acceleration.linear.setZ(
          statespace_filters_[2].getFilteredSecondDerivative());
      break;

    case FilterType::Butterworth:
      // TODO: Not supported yet
      state.acceleration.linear = {std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::quiet_NaN()};
      break;

    default:
      std::cerr << "KinematicStateFilter Error: computeLinearAcceleration()"
                << std::endl;
      break;
  }
}

void hiros::skeletons::KinematicStateFilter::computeOrientation(
    hiros::skeletons::types::KinematicState& state, const double& time) {
  // Init
  if (std::isnan(last_time_)) {
    last_time_ = time;
    last_state_ = state;

    if (skeletons::utils::isNaN(last_state_.velocity.angular)) {
      last_state_.velocity.angular = skeletons::types::Vector3(0, 0, 0);
    }

    if (skeletons::utils::isNaN(last_state_.acceleration.angular)) {
      last_state_.velocity.angular = skeletons::types::Vector3(0, 0, 0);
    }

    return;
  }

  // Filter
  auto weight{std::clamp((time - last_time_) * cutoff_frequency_, 0., 1.)};
  last_state_.pose.orientation =
      last_state_.pose.orientation.slerp(state.pose.orientation, weight);

  state.pose.orientation = last_state_.pose.orientation;
  last_time_ = time;
}

void hiros::skeletons::KinematicStateFilter::computeAngularVelocity(
    hiros::skeletons::types::KinematicState& state,
    const hiros::skeletons::types::KinematicState& prev_state,
    const double& prev_time) {
  // Init
  if (skeletons::utils::isNaN(prev_state.velocity.angular)) {
    return;
  }

  // Compute velocities
  double delta_roll{}, delta_pitch{}, delta_yaw{};
  tf2::Vector3 ang_vel{};
  auto dt{last_time_ - prev_time};

  if (dt <= 0) {
    return;
  }

  tf2::Matrix3x3 m{last_state_.pose.orientation *
                   prev_state.pose.orientation.inverse()};
  m.getEulerYPR(delta_yaw, delta_pitch, delta_roll);

  ang_vel.setX(delta_roll / dt);
  ang_vel.setY(delta_pitch / dt);
  ang_vel.setZ(delta_yaw / dt);

  // Filter
  auto weight{std::clamp(dt * cutoff_frequency_, 0., 1.)};
  last_state_.velocity.angular =
      last_state_.velocity.angular.lerp(ang_vel, weight);

  state.velocity.angular = last_state_.velocity.angular;
}

void hiros::skeletons::KinematicStateFilter::computeAngularAcceleration(
    hiros::skeletons::types::KinematicState& state,
    const hiros::skeletons::types::KinematicState& prev_state,
    const double& prev_time) {
  // Init
  if (skeletons::utils::isNaN(prev_state.acceleration.angular)) {
    return;
  }

  // Compute accelerations
  auto dt{last_time_ - prev_time};

  if (dt <= 0) {
    return;
  }

  auto ang_acc{(last_state_.velocity.angular - prev_state.velocity.angular) /
               dt};

  // Filter
  auto weight{std::clamp(dt * cutoff_frequency_, 0., 1.)};
  last_state_.acceleration.angular =
      last_state_.acceleration.angular.lerp(ang_acc, weight);

  state.acceleration.angular = last_state_.acceleration.angular;
}
