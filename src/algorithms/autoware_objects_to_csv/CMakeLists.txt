cmake_minimum_required(VERSION 3.5)
project(objects_to_csv)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
include_directories(include)

# Add executable
add_executable(tracked_objects_to_csv src/trackedobjectstocsv.cpp)

# Link libraries
ament_target_dependencies(tracked_objects_to_csv
  rclcpp
  autoware_perception_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# Install targets
install(TARGETS
  tracked_objects_to_csv
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()