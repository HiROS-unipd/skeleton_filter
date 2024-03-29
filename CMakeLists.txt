cmake_minimum_required(VERSION 3.5)
project(hiros_skeleton_filter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hiros_skeleton_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(Filter REQUIRED)

set(HEADER_FILES
    include/skeleton_filter/KinematicStateFilter.h
    include/skeleton_filter/SkeletonFilter.h
)

set(SOURCE_FILES
    src/KinematicStateFilter.cpp
    src/SkeletonFilter.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(skeleton_filter ${SOURCE_FILES} ${HEADER_FILES})
target_include_directories(skeleton_filter PRIVATE include)
ament_target_dependencies(skeleton_filter hiros_skeleton_msgs tf2)
ament_export_targets(skeleton_filter HAS_LIBRARY_TARGET)
ament_export_libraries(skeleton_filter)
ament_export_include_directories(include)
ament_export_dependencies(ament_cmake hiros_skeleton_msgs tf2)

add_executable(${PROJECT_NAME}_node
    src/main.cpp
    src/Filter.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)
target_link_libraries(${PROJECT_NAME}_node skeleton_filter)

set_target_properties(${PROJECT_NAME}_node
  PROPERTIES OUTPUT_NAME
  ${PROJECT_NAME}
  PREFIX
  ""
)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE include
)

install(
  DIRECTORY include/skeleton_filter
  DESTINATION include
)

install(
  TARGETS skeleton_filter
  EXPORT skeleton_filter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
