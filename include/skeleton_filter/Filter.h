#ifndef hiros_skeleton_filter_Filter_h
#define hiros_skeleton_filter_Filter_h

// Standard dependencies
#include <map>

// ROS dependencies
#include <ros/ros.h>

// Custom external dependencies
#include "hiros_skeleton_msgs/SkeletonGroup.h"
#include "rtb/Filter/StateSpaceFilter.h"
#include "skeletons/types.h"

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

    struct StateSpaceFilter3D
    {
      void filter(hiros::skeletons::types::Point& point, const double& time, const double& cutoff);

      std::array<rtb::Filter::StateSpaceFilter<double>, 3> filters;
    };

    class Filter
    {
    public:
      Filter() {}
      Filter(hiros::skeletons::types::Skeleton& skeleton, const double& cutoff);
      ~Filter() {}

      void configure();
      void start();

    private:
      void stop();
      void setupRosTopics();

      void callback(const hiros_skeleton_msgs::SkeletonGroup& sg);

      void init(hiros::skeletons::types::Skeleton& skeleton);
      void eraseUnusedFilters();

      void filter(hiros::skeletons::types::Skeleton& skeleton);
      void filter();

      ros::NodeHandle nh_{"~"};
      Parameters params_{};
      ros::Subscriber in_skeleton_group_sub{};
      ros::Publisher out_skeleton_group_pub{};

      // map<marker_id, marker_filter>
      typedef std::map<int, StateSpaceFilter3D> MarkerFiltersMap;
      // map<marker_group_id, marker_group_filter>
      typedef std::map<int, MarkerFiltersMap> MarkerGroupFiltersMap;
      // map<skeleton_id, skeleton_filter>
      typedef std::map<int, MarkerGroupFiltersMap> SkeletonFiltersMap;
      SkeletonFiltersMap filters_{};

      hiros::skeletons::types::SkeletonGroup skeleton_group_{};

      bool configured_{false};
    };

  } // namespace skeletons
} // namespace hiros

#endif
