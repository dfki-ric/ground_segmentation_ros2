cmake_minimum_required(VERSION 3.8)
project(ground_detection)

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

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Create a library target
add_library(ground_detection src/ground_detection.cpp)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(ground_detection PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen)

target_include_directories(ground_detection PUBLIC
                                            ${CMAKE_SOURCE_DIR}/include
                                            ${PCL_INCLUDE_DIRS}
                                            ${EIGEN_INCLUDE_DIR})

add_executable(ground_detection_node src/ground_detection_node.cpp)

target_include_directories(ground_detection_node PUBLIC
        ${pcl_conversions_INCLUDE_DIRS}
)


target_link_libraries(ground_detection_node
        ground_detection
        rclcpp::rclcpp
        tf2_ros::tf2_ros
        ${PCL_LIBRARIES}
        ${sensor_msgs_TARGETS}
        ${std_srvs_TARGETS}
        tf2_eigen::tf2_eigen
)

install(TARGETS ground_detection_node ground_detection
  DESTINATION lib/${PROJECT_NAME})

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
