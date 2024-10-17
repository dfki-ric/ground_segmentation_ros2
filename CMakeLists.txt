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

rosidl_generate_interfaces("ground_segmentation"
  "msg/GridCell.msg"
  "msg/GridMap.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(ground_segmentation_node src/ground_segmentation_node.cpp)

target_include_directories(ground_segmentation_node PUBLIC
        ${pcl_conversions_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/include
)

rosidl_target_interfaces(ground_segmentation_node "ground_segmentation" "rosidl_typesupport_cpp")

set(dependencies
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  pcl_conversions
  rviz_common
  rviz_rendering
  pluginlib
  message_filters
)

ament_target_dependencies(ground_segmentation_node
  ${dependencies}
)

target_link_libraries(ground_segmentation_node
        PkgConfig::pointcloud_obstacle_detection
        ${PCL_LIBRARIES}
)

qt5_wrap_cpp(MOC_FILES include/grid_map_plugin/GridMapDisplay.hpp)
# Create the plugin library
add_library(grid_map_plugin SHARED src/GridMapDisplay.cpp ${MOC_FILES})
rosidl_target_interfaces(grid_map_plugin "ground_segmentation" "rosidl_typesupport_cpp")
ament_target_dependencies(grid_map_plugin  ${dependencies})
target_include_directories(grid_map_plugin PUBLIC 
  #${ugv_nav4d-qt5_INCLUDE_DIRS} 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/grid_map_plugin/>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS grid_map_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS ground_segmentation_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
