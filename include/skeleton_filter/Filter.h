#ifndef hiros_skeleton_filter_Filter_h
#define hiros_skeleton_filter_Filter_h

// Standard dependencies
#include <map>

// ROS dependencies
#include <ros/ros.h>

// Custom external dependencies
#include "hiros_skeleton_msgs/SkeletonGroup.h"
#include "skeletons/types.h"

// Internal dependencies
#include "skeleton_filter/SkeletonFilter.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace skeletons {

    struct Parameters
    {
      std::string input_topic;
      std::string output_topic;
      double cutoff_frequency;
    };

    class Filter
    {
    public:
      Filter() {}
      ~Filter() {}

      void configure();
      void start();

    private:
      void stop();
      void setupRosTopics();

      void callback(const hiros_skeleton_msgs::SkeletonGroup& msg);

      void filter();
      void publish();

      void eraseUnusedFilters();
      void filter(hiros::skeletons::types::Skeleton& skeleton);

      void init(hiros::skeletons::types::Skeleton& skeleton);

      ros::NodeHandle nh_{"~"};
      Parameters params_{};

      ros::Subscriber in_skeleton_group_sub{};
      ros::Publisher out_skeleton_group_pub{};

      hiros::skeletons::types::SkeletonGroup skeleton_group_{};

      // map<skeleton_id, skeleton_filter>
      std::map<int, SkeletonFilter> filters_{};

      bool configured_{false};
    };

  } // namespace skeletons
} // namespace hiros

#endif
