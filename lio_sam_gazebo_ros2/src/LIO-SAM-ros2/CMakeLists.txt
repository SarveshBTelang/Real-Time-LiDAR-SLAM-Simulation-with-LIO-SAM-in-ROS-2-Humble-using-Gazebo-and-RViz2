cmake_minimum_required(VERSION 3.5)
project(lio_sam)

# Default to Release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
include_directories(PCL_CONVERSIONS_INCLUDE_DIRS)
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree sample_consensus segmentation visualization)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudInfo.msg"
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

# Node: featureExtraction
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
ament_target_dependencies(${PROJECT_NAME}_featureExtraction
  rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_msgs visualization_msgs
  tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL pcl_conversions
)
rosidl_target_interfaces(${PROJECT_NAME}_featureExtraction
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Node: imageProjection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
ament_target_dependencies(${PROJECT_NAME}_imageProjection
  rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_msgs visualization_msgs
  tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL pcl_conversions
)
rosidl_target_interfaces(${PROJECT_NAME}_imageProjection
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Node: imuPreintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration
  rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_msgs visualization_msgs
  tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM Eigen3 pcl_conversions
)
target_link_libraries(${PROJECT_NAME}_imuPreintegration gtsam)
rosidl_target_interfaces(${PROJECT_NAME}_imuPreintegration
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Node: mapOptimization
add_executable(${PROJECT_NAME}_mapOptimization src/mapOptmization.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapOptimization
  rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_msgs visualization_msgs
  tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM pcl_conversions
)
target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam)
rosidl_target_interfaces(${PROJECT_NAME}_mapOptimization
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install executables
install(TARGETS
  ${PROJECT_NAME}_featureExtraction
  ${PROJECT_NAME}_imageProjection
  ${PROJECT_NAME}_imuPreintegration
  ${PROJECT_NAME}_mapOptimization
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch/config
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# Export include directories
ament_export_include_directories(include)

# Linting and tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
