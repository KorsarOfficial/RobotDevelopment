cmake_minimum_required(VERSION 3.5)

project(manipulator_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoalPose.srv"
  "srv/GripperCmd.srv"
  "srv/JointPosition.srv"
  "srv/GripperDownPose.srv"
  "srv/SetInt32Array.srv"
  "srv/SetInt32.srv"
  DEPENDENCIES geometry_msgs 
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()
