// Internal dependencies
#include "skeleton_filter/Filter.h"

// Custom external dependencies
#include "skeletons/utils.h"

void hiros::skeletons::StateSpaceFilter3D::filter(hiros::skeletons::types::Point& point,
                                                  const double& time,
                                                  const double& cutoff)
{
  point.position.setX(filters[0].filter(point.position.x(), time, cutoff));
  point.position.setY(filters[1].filter(point.position.y(), time, cutoff));
  point.position.setZ(filters[2].filter(point.position.z(), time, cutoff));

  point.velocity.setX(filters[0].getFilteredFirstDerivative());
  point.velocity.setY(filters[1].getFilteredFirstDerivative());
  point.velocity.setZ(filters[2].getFilteredFirstDerivative());

  point.acceleration.setX(filters[0].getFilteredSecondDerivative());
  point.acceleration.setY(filters[1].getFilteredSecondDerivative());
  point.acceleration.setZ(filters[2].getFilteredSecondDerivative());
}

void hiros::skeletons::Filter::configure()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Filter... Configuring");

  if (configured_) {
    configured_ = false;
    stop();
  }

  nh_.getParam("input_topic", params_.input_topic);
  nh_.getParam("output_topic", params_.output_topic);
  nh_.getParam("cutoff_frequency", params_.cutoff_frequency);

  if (params_.cutoff_frequency <= 0) {
    ROS_FATAL_STREAM("Hi-ROS Skeleton Filter ERROR: Cutoff frequency <= 0. Unable to continue");
    stop();
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  setupRosTopics();

  configured_ = true;
  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Filter... CONFIGURED" << BASH_MSG_RESET);
}

void hiros::skeletons::Filter::start()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Filter... Starting");

  if (!configured_) {
    configure();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Filter... RUNNING" << BASH_MSG_RESET);
  ros::spin();
}

void hiros::skeletons::Filter::stop()
{
  ROS_INFO_STREAM("Hi-ROS Skeleton Filter... Stopping");

  if (in_skeleton_group_sub) {
    in_skeleton_group_sub.shutdown();
  }

  if (out_skeleton_group_pub) {
    out_skeleton_group_pub.shutdown();
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Hi-ROS Skeleton Filter... STOPPED" << BASH_MSG_RESET);
}

void hiros::skeletons::Filter::setupRosTopics()
{
  in_skeleton_group_sub = nh_.subscribe(params_.input_topic, 1, &Filter::callback, this);

  while (in_skeleton_group_sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
    ROS_WARN_STREAM_THROTTLE(
      2, "Hi-ROS Skeleton Filter WARNING: No input messages on input topic '" << params_.input_topic << "'");
  }

  out_skeleton_group_pub = nh_.advertise<hiros_skeleton_msgs::SkeletonGroup>(params_.output_topic, 1);
}

void hiros::skeletons::Filter::callback(const hiros_skeleton_msgs::SkeletonGroup& sg)
{
  skeleton_group_ = hiros::skeletons::utils::toStruct(sg);
  filter();
  out_skeleton_group_pub.publish(hiros::skeletons::utils::toMsg(skeleton_group_));
}

void hiros::skeletons::Filter::init(hiros::skeletons::types::Skeleton& skeleton)
{
  if (filters_.count(skeleton.id) > 0) {
    filters_.erase(skeleton.id);
  }

  for (auto& mkg : skeleton.marker_groups) {
    for (auto& mk : mkg.markers) {
      filters_[skeleton.id][mkg.id][mk.id] = StateSpaceFilter3D();
      filters_[skeleton.id][mkg.id][mk.id].filter(mk.point, skeleton_group_.time, params_.cutoff_frequency);
    }
  }
}

void hiros::skeletons::Filter::eraseUnusedFilters()
{
  std::erase_if(filters_, [&](const auto& pair) {
    const auto& [skeleton_id, skeleton_filter] = pair;
    return !skeleton_group_.hasSkeleton(skeleton_id);
  });
}

void hiros::skeletons::Filter::filter(hiros::skeletons::types::Skeleton& skeleton)
{
  if (filters_.count(skeleton.id) == 0) {
    init(skeleton);
  }

  for (auto& mkg_filters_pair : filters_.at(skeleton.id)) {
    auto& mkg_id = mkg_filters_pair.first;
    auto& mkg_filters = mkg_filters_pair.second;

    if (skeleton.hasMarkerGroup(mkg_id)) {
      auto& mkg = skeleton.getMarkerGroup(mkg_id);

      for (const auto& mk_filters_pair : mkg_filters) {
        auto& mk_id = mk_filters_pair.first;

        // erase filters relative to markers that are no longer present in skeleton
        if (!mkg.hasMarker(mk_id)) {
          mkg_filters.erase(mk_id);
        }
      }
    }
    // erase filters relative to marker groups that are no longer present in skeleton
    else {
      filters_.at(skeleton.id).erase(mkg_id);
    }
  }

  for (auto& mkg : skeleton.marker_groups) {
    // add new filters relative to marker groups that were not present in skeleton before
    if (filters_.at(skeleton.id).count(mkg.id) == 0) {
      filters_.at(skeleton.id)[mkg.id] = {};
    }

    for (auto& mk : mkg.markers) {
      // add new filters relative to markers that were not present in skeleton before
      if (filters_.at(skeleton.id)[mkg.id].count(mk.id) == 0) {
        filters_.at(skeleton.id)[mkg.id][mk.id] = StateSpaceFilter3D();
      }

      // filter all the marker trajectories
      filters_.at(skeleton.id)[mkg.id][mk.id].filter(mk.point, skeleton_group_.time, params_.cutoff_frequency);
    }
  }
}

void hiros::skeletons::Filter::filter()
{
  eraseUnusedFilters();

  for (auto& skeleton : skeleton_group_.skeletons) {
    filter(skeleton);
  }
}
