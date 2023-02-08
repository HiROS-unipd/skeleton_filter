// Standard dependencies
// Internal dependencies
#include "skeleton_filter/Filter.h"

// Custom external dependencies
#include "skeletons/utils.h"

const std::map<std::string, hiros::skeletons::FilterType>
    hiros::skeletons::Filter::k_filter_str_to_type{
        {{"statespace", hiros::skeletons::FilterType::StateSpace},
         {"butterworth", hiros::skeletons::FilterType::Butterworth}}};

hiros::skeletons::Filter::Filter() : Node("hiros_skeleton_filter") { start(); }

hiros::skeletons::Filter::~Filter() { stop(); }

void hiros::skeletons::Filter::start() {
  configure();

  RCLCPP_INFO_STREAM(get_logger(),
                     BASH_MSG_GREEN << "Running" << BASH_MSG_RESET);
}

void hiros::skeletons::Filter::stop() const {
  RCLCPP_INFO_STREAM(get_logger(),
                     BASH_MSG_GREEN << "Stopped" << BASH_MSG_RESET);

  rclcpp::shutdown();
}

void hiros::skeletons::Filter::configure() {
  getParams();
  setupRosTopics();
}

void hiros::skeletons::Filter::getParams() {
  getParam("input_topic", params_.input_topic);
  getParam("output_topic", params_.output_topic);
  getParam("filter", params_.filter_type);
  getParam("butterworth_order", params_.butterworth_order);
  getParam("sample_frequency", params_.sample_frequency);
  getParam("cutoff_frequency", params_.cutoff_frequency);

  if (k_filter_str_to_type.count(params_.filter_type) <= 0) {
    RCLCPP_FATAL_STREAM(
        get_logger(), "Filter type '" << params_.filter_type
                                      << "' not supported. Unable to continue");
    stop();
    exit(EXIT_FAILURE);
  }

  if (params_.filter_type == "butterworth" && params_.butterworth_order <= 0) {
    RCLCPP_FATAL_STREAM(get_logger(),
                        "Butterworth order <= 0. Unable to continue");
    stop();
    exit(EXIT_FAILURE);
  }

  if (params_.filter_type == "butterworth" && params_.sample_frequency <= 0) {
    RCLCPP_FATAL_STREAM(get_logger(),
                        "Sample frequency <= 0. Unable to continue");
    stop();
    exit(EXIT_FAILURE);
  }

  if (params_.cutoff_frequency <= 0) {
    RCLCPP_FATAL_STREAM(get_logger(),
                        "Cutoff frequency <= 0. Unable to continue");
    stop();
    exit(EXIT_FAILURE);
  }
}

void hiros::skeletons::Filter::setupRosTopics() {
  sub_ = create_subscription<hiros_skeleton_msgs::msg::SkeletonGroup>(
      params_.input_topic, 10,
      std::bind(&Filter::callback, this, std::placeholders::_1));

  pub_ = create_publisher<hiros_skeleton_msgs::msg::SkeletonGroup>(
      params_.output_topic, 10);
}

void hiros::skeletons::Filter::filter() {
  eraseUnusedFilters();

  for (auto& skeleton : skeleton_group_.skeletons) {
    filter(skeleton);
  }
}

void hiros::skeletons::Filter::filter(
    hiros::skeletons::types::Skeleton& skeleton) {
  if (filters_.count(skeleton.id) == 0) {
    initFilters(skeleton);
  }

  filters_.at(skeleton.id).filter(skeleton);
}

void hiros::skeletons::Filter::initFilters(
    hiros::skeletons::types::Skeleton& skeleton) {
  if (filters_.count(skeleton.id) > 0) {
    filters_.erase(skeleton.id);
  }

  filters_.emplace(
      skeleton.id,
      SkeletonFilter(skeleton, k_filter_str_to_type.at(params_.filter_type),
                     params_.cutoff_frequency, params_.sample_frequency,
                     static_cast<unsigned int>(params_.butterworth_order)));
}

void hiros::skeletons::Filter::eraseUnusedFilters() {
  for (auto it{filters_.begin()}; it != filters_.end();) {
    if (!skeleton_group_.hasSkeleton(it->first)) {
      it = filters_.erase(it);
    } else {
      ++it;
    }
  }
}

void hiros::skeletons::Filter::publish() {
  skeleton_group_.time = now().seconds();
  pub_->publish(hiros::skeletons::utils::toMsg(skeleton_group_));
}

void hiros::skeletons::Filter::callback(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg) {
  if (!rclcpp::ok()) {
    stop();
    exit(EXIT_FAILURE);
  }

  if (msg.skeletons.empty()) {
    return;
  }

  skeleton_group_ = hiros::skeletons::utils::toStruct(msg);

  filter();
  publish();
}
