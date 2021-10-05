#ifndef hiros_skeleton_filter_SkeletonFilter_h
#define hiros_skeleton_filter_SkeletonFilter_h

// Standard dependencies
#include <map>

// Internal dependencies
#include "skeleton_filter/KinematicStateFilter.h"

namespace hiros {
  namespace skeletons {

    class SkeletonFilter
    {
    public:
      SkeletonFilter() {}
      SkeletonFilter(hiros::skeletons::types::Skeleton& skeleton, const double& cutoff);
      ~SkeletonFilter() {}

      // Get filtered value and update the filter's internal state
      void filter(hiros::skeletons::types::Skeleton& skeleton,
                  const double& cutoff = std::numeric_limits<double>::quiet_NaN());

      // Get filtered value without modifying the filter's internal state
      void filter(hiros::skeletons::types::Skeleton& skeleton,
                  const double& cutoff = std::numeric_limits<double>::quiet_NaN()) const;

    private:
      void init(hiros::skeletons::types::Skeleton& skeleton, const double& cutoff);

      void updateMarkerFilters(hiros::skeletons::types::Skeleton& t_skeleton);
      void updateLinkFilters(hiros::skeletons::types::Skeleton& t_skeleton);

      // map<marker_id, marker_filter>
      std::map<int, KinematicStateFilter> marker_filters_{};
      // map<link_id, link_filter>
      std::map<int, KinematicStateFilter> link_filters_{};

      double cutoff_{};
      bool initialized_{false};
    };

  } // namespace skeletons
} // namespace hiros

#endif
