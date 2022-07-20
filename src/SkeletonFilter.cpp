// Custom external dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "skeleton_filter/SkeletonFilter.h"

const double k_epsilon = 1e-4;

hiros::skeletons::SkeletonFilter::SkeletonFilter(hiros::skeletons::types::Skeleton& skeleton,
                                                 const Type& filter_type,
                                                 const double& cutoff_frequency,
                                                 const double& sample_frequency,
                                                 const unsigned int& butterworth_order)
  : filter_type_(filter_type)
  , cutoff_frequency_(cutoff_frequency)
  , sample_frequency_(sample_frequency)
  , butterworth_order_(butterworth_order)
{
  if (filter_type == Type::Undefined) {
    std::cerr << "SkeletonFilter Error: Undefined filter type" << std::endl;
    exit(EXIT_FAILURE);
  }

  if (cutoff_frequency_ <= 0 || std::isnan(cutoff_frequency_)) {
    std::cerr << "SkeletonFilter Error: cutoff frequency out of range. Must be > 0" << std::endl;
    exit(EXIT_FAILURE);
  }

  if (filter_type_ == Type::Butterworth && (sample_frequency_ <= 0 || std::isnan(sample_frequency_))) {
    std::cerr << "SkeletonFilter Error: sampling frequency out of range. Must be > 0" << std::endl;
    exit(EXIT_FAILURE);
  }

  if (filter_type_ == Type::Butterworth && butterworth_order_ <= 0) {
    std::cerr << "SkeletonFilter Error: Butterworth filter order out of range. Must be in > 0" << std::endl;
    exit(EXIT_FAILURE);
  }

  init(skeleton);
}

void hiros::skeletons::SkeletonFilter::init(hiros::skeletons::types::Skeleton& skeleton)
{
  if (!initialized_) {
    for (auto& mk : skeleton.markers) {
      initMarkerFilter(mk, skeleton.src_time);
    }

    for (auto& lk : skeleton.links) {
      initLinkFilter(lk, skeleton.src_time);
    }

    initialized_ = true;
  }
}

void hiros::skeletons::SkeletonFilter::initMarkerFilter(hiros::skeletons::types::Marker& marker, const double& time)
{
  hiros::skeletons::types::KinematicState marker_center_copy;

  marker_filters_[marker.id] =
    KinematicStateFilter(filter_type_, cutoff_frequency_, sample_frequency_, butterworth_order_);

  // Hack to always have initial filtered value ~ initial raw value
  do {
    marker_center_copy = marker.center;
    marker_filters_[marker.id].filter(marker_center_copy, time);
  } while (!converged(marker.center, marker_center_copy, k_epsilon));
  marker.center = marker_center_copy;
}

void hiros::skeletons::SkeletonFilter::initLinkFilter(hiros::skeletons::types::Link& link, const double& time)
{
  hiros::skeletons::types::KinematicState link_center_copy;

  link_filters_[link.id] = KinematicStateFilter(filter_type_, cutoff_frequency_, sample_frequency_, butterworth_order_);

  // Hack to always have initial filtered value ~ initial raw value
  do {
    link_center_copy = link.center;
    link_filters_[link.id].filter(link_center_copy, time);
  } while (!converged(link.center, link_center_copy, k_epsilon));
  link.center = link_center_copy;
}

bool hiros::skeletons::SkeletonFilter::converged(const hiros::skeletons::types::KinematicState& lhs,
                                                 const hiros::skeletons::types::KinematicState& rhs,
                                                 const double& epsilon) const
{
  if (utils::isNaN(lhs.pose) || utils::isNaN(rhs.pose)) {
    return true;
  }

  bool converged = true;

  if (!utils::isNaN(lhs.pose.position) && !utils::isNaN(lhs.pose.position)) {
    if (utils::distance(lhs.pose.position, rhs.pose.position) / utils::magnitude(lhs.pose.position) > epsilon) {
      converged = false;
    }
  }

  if (!utils::isNaN(lhs.pose.orientation) && !utils::isNaN(lhs.pose.orientation)) {
    if (utils::distance(lhs.pose.orientation, rhs.pose.orientation) > epsilon) {
      converged = false;
    }
  }

  return converged;
}

void hiros::skeletons::SkeletonFilter::updateMarkerFilters(hiros::skeletons::types::Skeleton& skeleton)
{
  // erase empty markers
  std::erase_if(marker_filters_, [&](const auto& pair) { return !skeleton.hasMarker(pair.first); });

  for (auto& mk : skeleton.markers) {
    if (marker_filters_.count(mk.id) == 0) {
      // add new markers
      initMarkerFilter(mk, skeleton.src_time);
    }
    else {
      marker_filters_[mk.id].filter(mk.center, skeleton.src_time);
    }
  }
}

void hiros::skeletons::SkeletonFilter::updateLinkFilters(hiros::skeletons::types::Skeleton& skeleton)
{
  // erase empty links
  std::erase_if(link_filters_, [&](const auto& pair) { return !skeleton.hasLink(pair.first); });

  for (auto& lk : skeleton.links) {
    if (link_filters_.count(lk.id) == 0) {
      // new link
      initLinkFilter(lk, skeleton.src_time);
    }
    else {
      link_filters_[lk.id].filter(lk.center, skeleton.src_time);
    }
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
