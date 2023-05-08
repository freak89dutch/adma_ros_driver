cmake_minimum_required(VERSION 3.5)
project(adma_tools_cpp)

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
find_package(std_msgs REQUIRED)
find_package(adma_ros_driver_msgs REQUIRED)
find_package(adma_ros2_driver REQUIRED)

SET(PACKAGE_DEBS
  rclcpp
  sensor_msgs
  std_msgs
  adma_ros_driver_msgs
  adma_ros2_driver
)

include_directories(include)

add_library(${PROJECT_NAME}_node SHARED
  src/bag2gsdb_converter.cpp
  src/gsdb_server.cpp
  src/gsda_server.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  ${PACKAGE_DEBS}
)

add_executable(bag2gsdb_converter src/bag2gsdb_converter_main.cpp)
target_link_libraries(bag2gsdb_converter ${PROJECT_NAME}_node)
ament_target_dependencies(bag2gsdb_converter
  ${PACKAGE_DEBS}
)

add_executable(gsdb_server src/gsdb_server_main.cpp)
target_link_libraries(gsdb_server ${PROJECT_NAME}_node)
ament_target_dependencies(gsdb_server
  ${PACKAGE_DEBS}
)

add_executable(gsda_server src/gsda_server_main.cpp)
target_link_libraries(gsda_server ${PROJECT_NAME}_node)
ament_target_dependencies(gsda_server
  ${PACKAGE_DEBS}
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  bag2gsdb_converter
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  gsdb_server
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  gsda_server
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
