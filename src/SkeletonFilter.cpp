// Custom external dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_filter/SkeletonFilter.h"

hiros::skeletons::SkeletonFilter::SkeletonFilter(hiros::skeletons::types::Skeleton& skeleton,
                                                 const double& cutoff_frequency)
  : cutoff_frequency_(cutoff_frequency)
{
  init(skeleton);
}

void hiros::skeletons::SkeletonFilter::init(hiros::skeletons::types::Skeleton& skeleton)
{
  if (!initialized_) {
    for (auto& mk : skeleton.markers) {
      marker_filters_[mk.id] = KinematicStateFilter(cutoff_frequency_);
      marker_filters_[mk.id].filter(mk.center, skeleton.src_time);
    }

    for (auto& lk : skeleton.links) {
      link_filters_[lk.id] = KinematicStateFilter(cutoff_frequency_);
      link_filters_[lk.id].filter(lk.center, skeleton.src_time);
    }

    initialized_ = true;
  }
}

void hiros::skeletons::SkeletonFilter::updateMarkerFilters(hiros::skeletons::types::Skeleton& skeleton)
{
  // erase empty markers
  std::erase_if(marker_filters_, [&](const auto& pair) { return !skeleton.hasMarker(pair.first); });

  for (auto& mk : skeleton.markers) {
    if (marker_filters_.count(mk.id) == 0) {
      // add new markers
      marker_filters_[mk.id] = KinematicStateFilter(cutoff_frequency_);
    }
    marker_filters_[mk.id].filter(mk.center, skeleton.src_time);
  }
}

void hiros::skeletons::SkeletonFilter::updateLinkFilters(hiros::skeletons::types::Skeleton& skeleton)
{
  // erase empty links
  std::erase_if(link_filters_, [&](const auto& pair) { return !skeleton.hasLink(pair.first); });

  for (auto& lk : skeleton.links) {
    if (link_filters_.count(lk.id) == 0) {
      // new link
      link_filters_[lk.id] = KinematicStateFilter(cutoff_frequency_);
    }
    link_filters_[lk.id].filter(lk.center, skeleton.src_time);
  }
}

void hiros::skeletons::SkeletonFilter::filter(hiros::skeletons::types::Skeleton& skeleton)
{
  if (!initialized_) {
    init(skeleton);
  }

  updateMarkerFilters(skeleton);
  updateLinkFilters(skeleton);

  skeleton.bounding_box = skeletons::utils::computeBoundingBox(skeleton);
}

void hiros::skeletons::SkeletonFilter::filter(hiros::skeletons::types::Skeleton& skeleton) const
{
  auto tmp = *this;
  tmp.filter(skeleton);
}
