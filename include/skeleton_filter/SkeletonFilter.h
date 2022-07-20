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
      SkeletonFilter(hiros::skeletons::types::Skeleton& skeleton,
                     const Type& filter_type,
                     const double& cutoff_frequency,
                     const double& sample_frequency = 0,
                     const unsigned int& butterworth_order = 0);
      ~SkeletonFilter() {}

      inline void setFilterType(const Type& type) { filter_type_ = type; }
      inline Type getFilterType() const { return filter_type_; }

      // Get filtered value and update the filter's internal state
      void filter(hiros::skeletons::types::Skeleton& skeleton);

      // Get filtered value without modifying the filter's internal state
      void filter(hiros::skeletons::types::Skeleton& skeleton) const;

    private:
      void init(hiros::skeletons::types::Skeleton& skeleton);

      void updateMarkerFilters(hiros::skeletons::types::Skeleton& skeleton);
      void updateLinkFilters(hiros::skeletons::types::Skeleton& skeleton);

      // map<marker_id, marker_filter>
      std::map<int, KinematicStateFilter> marker_filters_{};
      // map<link_id, link_filter>
      std::map<int, KinematicStateFilter> link_filters_{};

      Type filter_type_{Type::Undefined};
      double cutoff_frequency_{};
      double sample_frequency_{};
      unsigned int butterworth_order_{};
      bool initialized_{false};
    };

  } // namespace skeletons
} // namespace hiros

#endif
