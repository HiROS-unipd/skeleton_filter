#include "skeleton_filter/Filter.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hiros::skeletons::Filter>());
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
