#ifndef hiros_skeleton_filter_SkeletonFilter_h
#define hiros_skeleton_filter_SkeletonFilter_h

// Standard dependencies
#include <map>

// Internal dependencies
#include "skeleton_filter/MarkerFilter.h"
#include "skeleton_filter/OrientationFilter.h"

namespace hiros {
  namespace skeletons {

    class SkeletonFilter
    {
    public:
      SkeletonFilter() {}
      SkeletonFilter(hiros::skeletons::types::Skeleton& skeleton, const double& cutoff);
      ~SkeletonFilter() {}

      void filter(hiros::skeletons::types::Skeleton& skeleton, const double& cutoff);
      void filter(hiros::skeletons::types::Skeleton& skeleton, const double& cutoff) const;

    private:
      void init(hiros::skeletons::types::Skeleton& skeleton, const double& cutoff);

      void updateMarkerFilters(hiros::skeletons::types::Skeleton& t_skeleton, const double& cutoff);
      void updateOrientationFilters(hiros::skeletons::types::Skeleton& t_skeleton, const double& cutoff);

      // map<marker_group_id, <marker_id, marker_filter>>
      std::map<int, std::map<int, MarkerFilter>> marker_filters_{};
      // map<orientation_group_id, <orientation_id, orientation_filter>>
      std::map<int, std::map<int, OrientationFilter>> orientation_filters_{};

      bool initialized_{false};
    };

  } // namespace skeletons
} // namespace hiros

#endif
