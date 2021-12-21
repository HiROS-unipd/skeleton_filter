// ROS dependencies
#include <tf2/LinearMath/Matrix3x3.h>

// Custom external dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_filter/KinematicStateFilter.h"

void hiros::skeletons::KinematicStateFilter::filter(hiros::skeletons::types::KinematicState& state,
                                                    const double& time,
                                                    const double& cutoff)
{
  auto prev_time = m_last_time;
  auto prev_state = m_last_state;

  if (!skeletons::utils::isNaN(state.pose.position)) {
    state.pose.position.setX(pos_filters_[0].filter(state.pose.position.x(), time, cutoff));
    state.pose.position.setY(pos_filters_[1].filter(state.pose.position.y(), time, cutoff));
    state.pose.position.setZ(pos_filters_[2].filter(state.pose.position.z(), time, cutoff));

    state.velocity.linear.setX(pos_filters_[0].getFilteredFirstDerivative());
    state.velocity.linear.setY(pos_filters_[1].getFilteredFirstDerivative());
    state.velocity.linear.setZ(pos_filters_[2].getFilteredFirstDerivative());

    state.acceleration.linear.setX(pos_filters_[0].getFilteredSecondDerivative());
    state.acceleration.linear.setY(pos_filters_[1].getFilteredSecondDerivative());
    state.acceleration.linear.setZ(pos_filters_[2].getFilteredSecondDerivative());
  }

  if (!skeletons::utils::isNaN(state.pose.orientation)) {
    computeOrientation(state, time, cutoff);
    computeAngularVelocity(state, prev_state, prev_time, cutoff);
    computeAngularAcceleration(state, prev_state, prev_time, cutoff);
  }
}

void hiros::skeletons::KinematicStateFilter::computeOrientation(hiros::skeletons::types::KinematicState& state,
                                                                const double& time,
                                                                const double& cutoff)
{
  // init
  if (std::isnan(m_last_time)) {
    m_last_time = time;
    m_last_state = state;

    if (skeletons::utils::isNaN(m_last_state.velocity.angular)) {
      m_last_state.velocity.angular = skeletons::types::Vector3(0, 0, 0);
    }

    if (skeletons::utils::isNaN(m_last_state.acceleration.angular)) {
      m_last_state.velocity.angular = skeletons::types::Vector3(0, 0, 0);
    }

    return;
  }

  // filter
  double weight = std::max(0., std::min((time - m_last_time) * cutoff, 1.));
  m_last_state.pose.orientation = m_last_state.pose.orientation.slerp(state.pose.orientation, weight);

  state.pose.orientation = m_last_state.pose.orientation;
  m_last_time = time;
}

void hiros::skeletons::KinematicStateFilter::computeAngularVelocity(
  hiros::skeletons::types::KinematicState& state,
  const hiros::skeletons::types::KinematicState& prev_state,
  const double& prev_time,
  const double& cutoff)
{
  // init
  if (skeletons::utils::isNaN(prev_state.velocity.angular)) {
    return;
  }

  // compute velocities
  double delta_roll, delta_pitch, delta_yaw;
  tf2::Vector3 ang_vel;
  double dt = m_last_time - prev_time;

  if (dt <= 0) {
    return;
  }

  tf2::Matrix3x3 m(m_last_state.pose.orientation * prev_state.pose.orientation.inverse());
  m.getEulerYPR(delta_yaw, delta_pitch, delta_roll);

  ang_vel.setX(delta_roll / dt);
  ang_vel.setY(delta_pitch / dt);
  ang_vel.setZ(delta_yaw / dt);

  // filter
  double weight = std::max(0., std::min(dt * cutoff, 1.));
  m_last_state.velocity.angular = m_last_state.velocity.angular.lerp(ang_vel, weight);

  state.velocity.angular = m_last_state.velocity.angular;
}

void hiros::skeletons::KinematicStateFilter::computeAngularAcceleration(
  hiros::skeletons::types::KinematicState& state,
  const hiros::skeletons::types::KinematicState& prev_state,
  const double& prev_time,
  const double& cutoff)
{
  // init
  if (skeletons::utils::isNaN(prev_state.acceleration.angular)) {
    return;
  }

  // compute accelerations
  double dt = m_last_time - prev_time;

  if (dt <= 0) {
    return;
  }

  auto ang_acc = (m_last_state.velocity.angular - prev_state.velocity.angular) / dt;

  // filter
  double weight = std::max(0., std::min(dt * cutoff, 1.));
  m_last_state.acceleration.angular = m_last_state.acceleration.angular.lerp(ang_acc, weight);

  state.acceleration.angular = m_last_state.acceleration.angular;
}
