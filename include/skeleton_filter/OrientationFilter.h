#ifndef hiros_skeleton_filter_OrientationFilter_h
#define hiros_skeleton_filter_OrientationFilter_h

// ROS dependencies
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

// Custom external dependencies
#include "skeletons/types.h"

namespace hiros {
  namespace skeletons {

    class OrientationFilter
    {
    public:
      void filter(hiros::skeletons::types::MIMU& mimu, const double& time, const double& cutoff);

    private:
      void computeOrientation(hiros::skeletons::types::MIMU& mimu, const double& time, const double& cutoff);
      void computeVelocity(hiros::skeletons::types::MIMU& mimu,
                           const tf2::Quaternion& prev_orientation,
                           const double& prev_time,
                           const double& cutoff);

      double last_time_{std::numeric_limits<double>::quiet_NaN()};
      tf2::Quaternion last_orientation_{};
      tf2::Vector3 last_velocity_{};
    };

  } // namespace skeletons
} // namespace hiros

#endif
