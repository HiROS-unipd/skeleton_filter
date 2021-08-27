// ROS
#include <ros/ros.h>

// Internal dependencies
#include "skeleton_filter/Filter.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "hiros_skeleton_filter");

  hiros::skeletons::Filter filter;
  filter.start();

  exit(EXIT_SUCCESS);
}
