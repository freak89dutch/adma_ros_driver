cmake_minimum_required(VERSION 3.0.2)
project(adma_tools_py)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

find_package(catkin REQUIRED COMPONENTS
  rospy
  adma_ros_driver_msgs
)

catkin_python_setup()

catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES adma_tools_py
  # CATKIN_DEPENDS rospy
  # DEPENDS system_lib
)

include_directories(

  # include
  ${catkin_INCLUDE_DIRS}
)
