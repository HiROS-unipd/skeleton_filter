#ifndef hiros_skeleton_filter_Filter_h
#define hiros_skeleton_filter_Filter_h

// Standard dependencies
#include <map>

// ROS dependencies
#include <rclcpp/rclcpp.hpp>

// Custom external dependencies
#include "hiros_skeleton_msgs/msg/skeleton_group.hpp"
#include "skeletons/types.h"

// Internal dependencies
#include "skeleton_filter/SkeletonFilter.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
namespace skeletons {

class Filter : public rclcpp::Node {
 public:
  Filter();
  ~Filter();

 private:
  static const std::map<std::string, FilterType> k_filter_str_to_type;

  struct Parameters {
    std::string input_topic{};
    std::string output_topic{};

    std::string filter_type{};
    int butterworth_order{};
    double sample_frequency{};
    double cutoff_frequency{};
  };

  template <typename T>
  bool getParam(const std::string& name, T& parameter) {
    declare_parameter<T>(name);
    return get_parameter(name, parameter);
  }

  void start();
  void stop() const;
  void configure();

  void getParams();
  void setupRosTopics();

  void filter();
  void filter(hiros::skeletons::types::Skeleton& skeleton);
  void initFilters(hiros::skeletons::types::Skeleton& skeleton);
  void eraseUnusedFilters();

  void publish();

  void callback(const hiros_skeleton_msgs::msg::SkeletonGroup& msg);

  rclcpp::Subscription<hiros_skeleton_msgs::msg::SkeletonGroup>::SharedPtr
      sub_{};
  rclcpp::Publisher<hiros_skeleton_msgs::msg::SkeletonGroup>::SharedPtr pub_{};

  Parameters params_{};

  hiros::skeletons::types::SkeletonGroup skeleton_group_{};
  // map<skeleton_id, skeleton_filter>
  std::map<int, SkeletonFilter> filters_{};
};

}  // namespace skeletons
}  // namespace hiros

#endif
