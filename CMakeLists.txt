cmake_minimum_required(VERSION 3.8)
project(ground_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)

pkg_check_modules(pointcloud_obstacle_detection IMPORTED_TARGET REQUIRED pointcloud_obstacle_detection)

add_executable(ground_segmentation_node src/ground_segmentation_node.cpp)

target_include_directories(ground_segmentation_node PUBLIC
        ${pcl_conversions_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/include
)

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
  visualization_msgs
  pcl_conversions
  message_filters
)

ament_target_dependencies(ground_segmentation_node
  ${dependencies}
)

target_include_directories(ground_segmentation_node PRIVATE
    ${pointcloud_obstacle_detection_INCLUDE_DIRS}
)

target_link_libraries(ground_segmentation_node
        ${PCL_LIBRARIES}
)

install(TARGETS ground_segmentation_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
