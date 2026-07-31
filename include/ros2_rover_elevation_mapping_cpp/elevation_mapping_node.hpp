#ifndef ROS2_ROVER_ELEVATION_MAPPING_CPP__ELEVATION_MAPPING_NODE_HPP_
#define ROS2_ROVER_ELEVATION_MAPPING_CPP__ELEVATION_MAPPING_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#if __has_include(<tf2_ros/buffer.hpp>)
  #include <tf2_ros/buffer.hpp>
#else
  #include <tf2_ros/buffer.h>
#endif

#if __has_include(<tf2_ros/transform_listener.hpp>)
  #include <tf2_ros/transform_listener.hpp>
#else
  #include <tf2_ros/transform_listener.h>
#endif

#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ElevationMappingNode : public rclcpp::Node
{
public:
  ElevationMappingNode();

private:
  void declareParameters();
  void loadParameters();

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  bool transformCloudToTargetFrame(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_msg,
    sensor_msgs::msg::PointCloud2 & output_msg);

  void filterByHeight(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & output);

  void downsampleCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & output);

  void estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    pcl::PointCloud<pcl::Normal>::Ptr & normals);

  void publishFilteredCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std_msgs::msg::Header & header);

  void publishTraversabilityGrid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr & normals,
    const std_msgs::msg::Header & header);

  bool getSensorPosition(
    const rclcpp::Time & stamp,
    double & sensor_x,
    double & sensor_y);

  bool worldToGrid(
    double world_x,
    double world_y,
    int & cell_x,
    int & cell_y) const;

  bool isInsideGrid(int cell_x, int cell_y) const;

  void raytraceFreeCells(
    std::vector<uint8_t> & observed_free,
    int start_x,
    int start_y,
    int end_x,
    int end_y) const;

  int slopeToCost(double slope_deg) const;
  int heightToCost(double point_z) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string input_cloud_topic_;
  std::string output_grid_topic_;
  std::string output_cloud_topic_;
  std::string target_frame_;
  std::string raytrace_sensor_frame_;

  bool raytrace_enabled_;
  double raytrace_min_range_;
  double raytrace_max_range_;
  int raytrace_endpoint_margin_cells_;

  double map_size_x_;
  double map_size_y_;
  double resolution_;

  double obstacle_min_height_;
  double lethal_obstacle_height_;
  double obstacle_max_height_;

  double min_z_;
  double max_z_;
  double voxel_leaf_size_;
  double normal_radius_;
  double max_slope_deg_;
  double hard_slope_deg_;
  double roughness_threshold_;

  int width_;
  int height_;
};

#endif  // ROS2_ROVER_ELEVATION_MAPPING_CPP__ELEVATION_MAPPING_NODE_HPP_
