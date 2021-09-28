// Internal dependencies
#include "skeleton_filter/SkeletonFilter.h"

hiros::skeletons::SkeletonFilter::SkeletonFilter(hiros::skeletons::types::Skeleton& skeleton, const double& cutoff)
{
  init(skeleton, cutoff);
}

void hiros::skeletons::SkeletonFilter::init(hiros::skeletons::types::Skeleton& skeleton, const double& cutoff)
{
  if (!initialized_) {
    for (auto& mkg : skeleton.marker_groups) {
      for (auto& mk : mkg.markers) {
        marker_filters_[mkg.id][mk.id] = MarkerFilter();
        marker_filters_[mkg.id][mk.id].filter(mk.point, skeleton.src_time, cutoff);
      }
    }

    initialized_ = true;
  }
}

void hiros::skeletons::SkeletonFilter::updateMarkerFilters(hiros::skeletons::types::Skeleton& skeleton,
                                                           const double& cutoff)
{
  for (const auto& mkg : marker_filters_) {
    auto mkg_id = mkg.first;

    if (skeleton.hasMarkerGroup(mkg_id)) {
      auto& mkg = skeleton.getMarkerGroup(mkg_id);

      for (const auto& mk : marker_filters_.at(mkg_id)) {
        auto mk_id = mk.first;

        if (!mkg.hasMarker(mk_id)) {
          // erase empty markers
          marker_filters_.at(mkg_id).erase(mk_id);
        }
      }
    }
    else {
      // erase empty marker groups
      marker_filters_.erase(mkg_id);
    }
  }

  for (auto& mkg : skeleton.marker_groups) {
    if (marker_filters_.count(mkg.id) == 0) {
      // add new marker groups
      marker_filters_[mkg.id] = {};
    }

    for (auto& mk : mkg.markers) {
      if (marker_filters_[mkg.id].count(mk.id) == 0) {
        // add new markers
        marker_filters_[mkg.id][mk.id] = MarkerFilter();
      }

      // filter
      marker_filters_[mkg.id][mk.id].filter(mk.point, skeleton.src_time, cutoff);
    }
  }
}

void hiros::skeletons::SkeletonFilter::filter(hiros::skeletons::types::Skeleton& skeleton, const double& cutoff)
{
  if (!initialized_) {
    if (cutoff < 0 || std::isnan(cutoff)) {
      std::cerr << "hiros::skeletons::Filter Warning: cutoff out of range. Must be in ]0, +inf[" << std::endl;
      return;
    }
    init(skeleton, cutoff);
  }

  updateMarkerFilters(skeleton, cutoff);
}

void hiros::skeletons::SkeletonFilter::filter(hiros::skeletons::types::Skeleton& skeleton, const double& cutoff) const
{
  auto tmp = *this;
  tmp.filter(skeleton, cutoff);
}
