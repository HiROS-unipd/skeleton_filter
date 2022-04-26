// ROS dependencies
#include <tf2/LinearMath/Matrix3x3.h>

// Custom external dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_filter/KinematicStateFilter.h"

hiros::skeletons::KinematicStateFilter::KinematicStateFilter(const double& cutoff_frequency)
  : cutoff_frequency_(cutoff_frequency){};

void hiros::skeletons::KinematicStateFilter::filter(hiros::skeletons::types::KinematicState& state, const double& time)
{
  auto prev_time = last_time_;
  auto prev_state = last_state_;

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

void hiros::skeletons::KinematicStateFilter::computePosition(hiros::skeletons::types::KinematicState& state,
                                                             const double& time)
{
  state.pose.position.setX(pos_filters_[0].filter(state.pose.position.x(), time, cutoff_frequency_));
  state.pose.position.setY(pos_filters_[1].filter(state.pose.position.y(), time, cutoff_frequency_));
  state.pose.position.setZ(pos_filters_[2].filter(state.pose.position.z(), time, cutoff_frequency_));
}

void hiros::skeletons::KinematicStateFilter::computeLinearVelocity(hiros::skeletons::types::KinematicState& state)
{
  state.velocity.linear.setX(pos_filters_[0].getFilteredFirstDerivative());
  state.velocity.linear.setY(pos_filters_[1].getFilteredFirstDerivative());
  state.velocity.linear.setZ(pos_filters_[2].getFilteredFirstDerivative());
}

void hiros::skeletons::KinematicStateFilter::computeLinearAcceleration(hiros::skeletons::types::KinematicState& state)
{
  state.acceleration.linear.setX(pos_filters_[0].getFilteredSecondDerivative());
  state.acceleration.linear.setY(pos_filters_[1].getFilteredSecondDerivative());
  state.acceleration.linear.setZ(pos_filters_[2].getFilteredSecondDerivative());
}

void hiros::skeletons::KinematicStateFilter::computeOrientation(hiros::skeletons::types::KinematicState& state,
                                                                const double& time)
{
  // init
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

  // filter
  double weight = std::max(0., std::min((time - last_time_) * cutoff_frequency_, 1.));
  last_state_.pose.orientation = last_state_.pose.orientation.slerp(state.pose.orientation, weight);

  state.pose.orientation = last_state_.pose.orientation;
  last_time_ = time;
}

void hiros::skeletons::KinematicStateFilter::computeAngularVelocity(
  hiros::skeletons::types::KinematicState& state,
  const hiros::skeletons::types::KinematicState& prev_state,
  const double& prev_time)
{
  // init
  if (skeletons::utils::isNaN(prev_state.velocity.angular)) {
    return;
  }

  // compute velocities
  double delta_roll, delta_pitch, delta_yaw;
  tf2::Vector3 ang_vel;
  double dt = last_time_ - prev_time;

  if (dt <= 0) {
    return;
  }

  tf2::Matrix3x3 m(last_state_.pose.orientation * prev_state.pose.orientation.inverse());
  m.getEulerYPR(delta_yaw, delta_pitch, delta_roll);

  ang_vel.setX(delta_roll / dt);
  ang_vel.setY(delta_pitch / dt);
  ang_vel.setZ(delta_yaw / dt);

  // filter
  double weight = std::max(0., std::min(dt * cutoff_frequency_, 1.));
  last_state_.velocity.angular = last_state_.velocity.angular.lerp(ang_vel, weight);

  state.velocity.angular = last_state_.velocity.angular;
}

void hiros::skeletons::KinematicStateFilter::computeAngularAcceleration(
  hiros::skeletons::types::KinematicState& state,
  const hiros::skeletons::types::KinematicState& prev_state,
  const double& prev_time)
{
  // init
  if (skeletons::utils::isNaN(prev_state.acceleration.angular)) {
    return;
  }

  // compute accelerations
  double dt = last_time_ - prev_time;

  if (dt <= 0) {
    return;
  }

  auto ang_acc = (last_state_.velocity.angular - prev_state.velocity.angular) / dt;

  // filter
  double weight = std::max(0., std::min(dt * cutoff_frequency_, 1.));
  last_state_.acceleration.angular = last_state_.acceleration.angular.lerp(ang_acc, weight);

  state.acceleration.angular = last_state_.acceleration.angular;
}
