cmake_minimum_required(VERSION 3.12)
project(ground_segmentation_ros2)

# Silence PCL's nested find_package(FLANN) CMP0144 dev warning; set as
# default since cmake_policy(SET) wouldn't propagate into that nested call.
if(POLICY CMP0144)
    set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ground_segmentation REQUIRED)

set(dependencies
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
  message_filters
)

add_executable(ground_segmentation_ros2_node src/ground_segmentation_ros2_node.cpp)
target_include_directories(ground_segmentation_ros2_node PUBLIC
  ${pcl_conversions_INCLUDE_DIRS}
)

ament_target_dependencies(ground_segmentation_ros2_node
  ${dependencies}
)

target_link_libraries(ground_segmentation_ros2_node
  ${PCL_LIBRARIES}
  ground_segmentation::ground_segmentation
)

install(TARGETS ground_segmentation_ros2_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # The sources do not follow the uncrustify style.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
