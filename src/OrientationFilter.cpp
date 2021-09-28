// ROS dependencies
#include <tf2/LinearMath/Matrix3x3.h>

// Internal dependencies
#include "skeleton_filter/OrientationFilter.h"

void hiros::skeletons::OrientationFilter::filter(hiros::skeletons::types::MIMU& mimu,
                                                 const double& time,
                                                 const double& cutoff)
{
  auto prev_orientation = last_orientation_;
  auto prev_time = last_time_;

  computeOrientation(mimu, time, cutoff);
  computeVelocity(mimu, prev_orientation, prev_time, cutoff);
}

void hiros::skeletons::OrientationFilter::computeOrientation(hiros::skeletons::types::MIMU& mimu,
                                                             const double& time,
                                                             const double& cutoff)
{
  // init
  if (std::isnan(last_time_)) {
    last_time_ = time;
    last_orientation_ = mimu.orientation;
    last_velocity_ = std::isnan(mimu.angular_velocity.x()) ? tf2::Vector3(0, 0, 0) : mimu.angular_velocity;
    return;
  }

  // filter
  double weight = std::max(0., std::min((time - last_time_) * cutoff, 1.));
  last_orientation_ = last_orientation_.slerp(mimu.orientation, weight);
  mimu.orientation = last_orientation_;
  last_time_ = time;
}

void hiros::skeletons::OrientationFilter::computeVelocity(hiros::skeletons::types::MIMU& mimu,
                                                          const tf2::Quaternion& prev_orientation,
                                                          const double& prev_time,
                                                          const double& cutoff)
{
  // init
  if (std::isnan(prev_time)) {
    mimu.angular_velocity = tf2::Vector3(0, 0, 0);
    return;
  }

  // compute angular velocities
  double delta_roll, delta_pitch, delta_yaw;
  tf2::Vector3 curr_vel;
  double dt = last_time_ - prev_time;

  if (dt > 0) {
    tf2::Matrix3x3 m(last_orientation_ * prev_orientation.inverse());
    m.getEulerYPR(delta_yaw, delta_pitch, delta_roll);

    curr_vel.setX(delta_roll / dt);
    curr_vel.setY(delta_pitch / dt);
    curr_vel.setZ(delta_yaw / dt);

    // filter
    double weight = std::max(0., std::min(dt * cutoff, 1.));
    last_velocity_ = last_velocity_.lerp(curr_vel, weight);
  }

  mimu.angular_velocity = last_velocity_;
}
