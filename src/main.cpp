#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_rover_elevation_mapping_cpp/elevation_mapping_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ElevationMappingNode>());
  rclcpp::shutdown();
  return 0;
}
