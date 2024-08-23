cmake_minimum_required(VERSION 3.5)
project(lidar_perception_with_diagnostics)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

# Add executable
add_executable(lidar_perception_with_diagnostics src/lidar_perception_with_diagnostics.cpp)

# Link libraries
ament_target_dependencies(lidar_perception_with_diagnostics
  rclcpp
  sensor_msgs
  std_msgs
  diagnostic_updater
)

# Install the executables
install(TARGETS
  lidar_perception_with_diagnostics
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
