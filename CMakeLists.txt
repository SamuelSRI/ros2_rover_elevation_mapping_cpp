cmake_minimum_required(VERSION 3.10)
project(ros2_rover_elevation_mapping_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters features search kdtree)

add_executable(elevation_mapping_node
  src/main.cpp
  src/elevation_mapping_node.cpp
)

target_include_directories(elevation_mapping_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(elevation_mapping_node ${PCL_LIBRARIES})

ament_target_dependencies(elevation_mapping_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_msgs
)

install(TARGETS elevation_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
