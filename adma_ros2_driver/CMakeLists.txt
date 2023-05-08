cmake_minimum_required(VERSION 3.5)
project(adma_ros2_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options() # (-Wall -Wextra) #-Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(adma_ros_driver_msgs REQUIRED)

SET(PACKAGE_DEBS
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  adma_ros_driver_msgs
)

include_directories(include)

add_library(${PROJECT_NAME}_node SHARED
  src/parser/adma2ros_parser.cpp
  src/parser/adma2ros_parser_v32.cpp
  src/parser/adma2ros_parser_v333.cpp
  src/parser/adma2ros_parser_v334.cpp
  src/parser/parser_utils.cpp
  src/adma_driver.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  ${PACKAGE_DEBS}
)



add_executable(adma_driver src/adma_driver.cpp)
target_link_libraries(adma_driver ${PROJECT_NAME}_node)
ament_target_dependencies(adma_driver
  ${PACKAGE_DEBS}
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  adma_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)

# install additional files of folders
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_node)

ament_package()
