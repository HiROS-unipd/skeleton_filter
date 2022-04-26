// Internal dependencies
#include "skeleton_filter/Filter.h"

// Custom external dependencies
#include "skeletons/utils.h"

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
  ros::shutdown();
}

void hiros::skeletons::Filter::setupRosTopics()
{
  in_skeleton_group_sub = nh_.subscribe(params_.input_topic, 1, &Filter::callback, this);

  while (in_skeleton_group_sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
    ROS_WARN_STREAM_DELAYED_THROTTLE(
      2, "Hi-ROS Skeleton Filter Warning: No input messages on input topic '" << params_.input_topic << "'");
  }

  out_skeleton_group_pub = nh_.advertise<hiros_skeleton_msgs::SkeletonGroup>(params_.output_topic, 1);
}

void hiros::skeletons::Filter::callback(const hiros_skeleton_msgs::SkeletonGroup& msg)
{
  if (!ros::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  skeleton_group_ = hiros::skeletons::utils::toStruct(msg);

  filter();
  publish();
}

void hiros::skeletons::Filter::filter()
{
  eraseUnusedFilters();

  for (auto& skeleton : skeleton_group_.skeletons) {
    filter(skeleton);
  }
}

void hiros::skeletons::Filter::publish()
{
  skeleton_group_.time = ros::Time::now().toSec();
  out_skeleton_group_pub.publish(hiros::skeletons::utils::toMsg(skeleton_group_));
}

void hiros::skeletons::Filter::eraseUnusedFilters()
{
  std::erase_if(filters_, [&](const auto& pair) { return !skeleton_group_.hasSkeleton(pair.first); });
}

void hiros::skeletons::Filter::filter(hiros::skeletons::types::Skeleton& skeleton)
{
  if (filters_.count(skeleton.id) == 0) {
    init(skeleton);
  }

  filters_.at(skeleton.id).filter(skeleton);
}

void hiros::skeletons::Filter::init(hiros::skeletons::types::Skeleton& skeleton)
{
  if (filters_.count(skeleton.id) > 0) {
    filters_.erase(skeleton.id);
  }

  filters_.emplace(skeleton.id, SkeletonFilter(skeleton, params_.cutoff_frequency));
}
