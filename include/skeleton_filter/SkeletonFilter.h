#ifndef hiros_skeleton_filter_SkeletonFilter_h
#define hiros_skeleton_filter_SkeletonFilter_h

// Standard dependencies
#include <map>

// Internal dependencies
#include "skeleton_filter/KinematicStateFilter.h"

namespace hiros {
namespace skeletons {

class SkeletonFilter {
 public:
  SkeletonFilter() {}
  SkeletonFilter(hiros::skeletons::types::Skeleton& skeleton,
                 const FilterType& filter_type, const double& cutoff_frequency,
                 const double& sample_frequency = 0,
                 const unsigned int& butterworth_order = 0);
  ~SkeletonFilter() {}

  inline void setFilterType(const FilterType& type) { filter_type_ = type; }
  inline FilterType getFilterType() const { return filter_type_; }

  // Get filtered value and update the filter's internal state
  void filter(hiros::skeletons::types::Skeleton& skeleton);

  // Get filtered value without modifying the filter's internal state
  void filter(hiros::skeletons::types::Skeleton& skeleton) const;

 private:
  void init(hiros::skeletons::types::Skeleton& skeleton);
  void initMarkerFilter(hiros::skeletons::types::Marker& marker,
                        const double& time);
  void initLinkFilter(hiros::skeletons::types::Link& link, const double& time);

  bool converged(const hiros::skeletons::types::KinematicState& lhs,
                 const hiros::skeletons::types::KinematicState& rhs,
                 const double& epsilon) const;

  void updateMarkerFilters(hiros::skeletons::types::Skeleton& skeleton);
  void updateLinkFilters(hiros::skeletons::types::Skeleton& skeleton);

  // map<marker_id, marker_filter>
  std::map<int, KinematicStateFilter> marker_filters_{};
  // map<link_id, link_filter>
  std::map<int, KinematicStateFilter> link_filters_{};

  FilterType filter_type_{FilterType::Undefined};
  double cutoff_frequency_{};
  double sample_frequency_{};
  unsigned int butterworth_order_{};
  bool initialized_{false};
};

}  // namespace skeletons
}  // namespace hiros

#endif
