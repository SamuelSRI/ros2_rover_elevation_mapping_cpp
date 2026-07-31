#include "ros2_rover_elevation_mapping_cpp/elevation_mapping_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#if __has_include(<tf2/time.hpp>)
  #include <tf2/time.hpp>
#else
  #include <tf2/time.h>
#endif

#if __has_include(<tf2_sensor_msgs/tf2_sensor_msgs.hpp>)
  #include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#else
  #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#endif

ElevationMappingNode::ElevationMappingNode()
: Node("elevation_mapping_node")
{
  declareParameters();
  loadParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_cloud_topic_, 10);

  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    output_grid_topic_, 10);

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ElevationMappingNode::cloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Elevation mapping node started");
  RCLCPP_INFO(this->get_logger(), "Input cloud: %s", input_cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Output grid: %s", output_grid_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "Raytrace origin: %s", raytrace_sensor_frame_.c_str());
}

void ElevationMappingNode::declareParameters()
{
  this->declare_parameter<std::string>("input_cloud_topic", "/merged_cloud");
  this->declare_parameter<std::string>("output_grid_topic", "/traversability_grid");
  this->declare_parameter<std::string>("output_cloud_topic", "/local_cloud_filtered");
  this->declare_parameter<std::string>("target_frame", "base_link");

  this->declare_parameter<bool>("raytrace_enabled", true);
  this->declare_parameter<std::string>("raytrace_sensor_frame", "as_dt1_link");
  this->declare_parameter<double>("raytrace_min_range", 0.30);
  this->declare_parameter<double>("raytrace_max_range", 7.0);
  this->declare_parameter<int>("raytrace_endpoint_margin_cells", 1);

  this->declare_parameter<double>("map_size_x", 10.0);
  this->declare_parameter<double>("map_size_y", 10.0);
  this->declare_parameter<double>("resolution", 0.10);

  this->declare_parameter<double>("min_z", -1.0);
  this->declare_parameter<double>("max_z", 2.0);
  this->declare_parameter<double>("voxel_leaf_size", 0.05);

  this->declare_parameter<double>("normal_radius", 0.25);
  this->declare_parameter<double>("max_slope_deg", 20.0);
  this->declare_parameter<double>("hard_slope_deg", 30.0);
  this->declare_parameter<double>("roughness_threshold", 0.08);

  this->declare_parameter<double>("obstacle_min_height", 0.10);
  this->declare_parameter<double>("lethal_obstacle_height", 0.20);
  this->declare_parameter<double>("obstacle_max_height", 1.50);
}

void ElevationMappingNode::loadParameters()
{
  input_cloud_topic_ = this->get_parameter("input_cloud_topic").as_string();
  output_grid_topic_ = this->get_parameter("output_grid_topic").as_string();
  output_cloud_topic_ = this->get_parameter("output_cloud_topic").as_string();
  target_frame_ = this->get_parameter("target_frame").as_string();

  raytrace_enabled_ = this->get_parameter("raytrace_enabled").as_bool();
  raytrace_sensor_frame_ = this->get_parameter("raytrace_sensor_frame").as_string();
  raytrace_min_range_ = this->get_parameter("raytrace_min_range").as_double();
  raytrace_max_range_ = this->get_parameter("raytrace_max_range").as_double();
  raytrace_endpoint_margin_cells_ =
    this->get_parameter("raytrace_endpoint_margin_cells").as_int();

  map_size_x_ = this->get_parameter("map_size_x").as_double();
  map_size_y_ = this->get_parameter("map_size_y").as_double();
  resolution_ = this->get_parameter("resolution").as_double();

  min_z_ = this->get_parameter("min_z").as_double();
  max_z_ = this->get_parameter("max_z").as_double();
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();

  normal_radius_ = this->get_parameter("normal_radius").as_double();
  max_slope_deg_ = this->get_parameter("max_slope_deg").as_double();
  hard_slope_deg_ = this->get_parameter("hard_slope_deg").as_double();
  roughness_threshold_ = this->get_parameter("roughness_threshold").as_double();

  obstacle_min_height_ = this->get_parameter("obstacle_min_height").as_double();
  lethal_obstacle_height_ = this->get_parameter("lethal_obstacle_height").as_double();
  obstacle_max_height_ = this->get_parameter("obstacle_max_height").as_double();

  resolution_ = std::max(resolution_, 0.001);
  map_size_x_ = std::max(map_size_x_, resolution_);
  map_size_y_ = std::max(map_size_y_, resolution_);
  raytrace_min_range_ = std::max(0.0, raytrace_min_range_);
  raytrace_max_range_ = std::max(raytrace_min_range_, raytrace_max_range_);
  raytrace_endpoint_margin_cells_ = std::max(0, raytrace_endpoint_margin_cells_);

  width_ = std::max(1, static_cast<int>(std::round(map_size_x_ / resolution_)));
  height_ = std::max(1, static_cast<int>(std::round(map_size_y_ / resolution_)));
}

void ElevationMappingNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 cloud_in_target_frame;
  if (!transformCloudToTargetFrame(msg, cloud_in_target_frame)) {
    return;
  }

  auto cloud_raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(cloud_in_target_frame, *cloud_raw);
  if (cloud_raw->empty()) {
    return;
  }

  auto cloud_z_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  filterByHeight(cloud_raw, cloud_z_filtered);

  auto cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  downsampleCloud(cloud_z_filtered, cloud_filtered);

  if (cloud_filtered->size() < 20) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Filtered cloud has too few points: %zu", cloud_filtered->size());
    return;
  }

  auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  estimateNormals(cloud_filtered, normals);

  publishFilteredCloud(cloud_filtered, cloud_in_target_frame.header);
  publishTraversabilityGrid(cloud_filtered, normals, cloud_in_target_frame.header);
}

bool ElevationMappingNode::transformCloudToTargetFrame(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_msg,
  sensor_msgs::msg::PointCloud2 & output_msg)
{
  if (target_frame_.empty() || input_msg->header.frame_id == target_frame_) {
    output_msg = *input_msg;
    return true;
  }

  try {
    output_msg = tf_buffer_->transform(
      *input_msg, target_frame_, tf2::durationFromSec(0.1));
    output_msg.header.frame_id = target_frame_;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Cannot transform cloud from '%s' to '%s': %s",
      input_msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
    return false;
  }
}

void ElevationMappingNode::filterByHeight(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & output)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_z_, max_z_);
  pass.filter(*output);
}

void ElevationMappingNode::downsampleCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & output)
{
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(input);
  voxel.setLeafSize(
    static_cast<float>(voxel_leaf_size_),
    static_cast<float>(voxel_leaf_size_),
    static_cast<float>(voxel_leaf_size_));
  voxel.filter(*output);
}

void ElevationMappingNode::estimateNormals(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  pcl::PointCloud<pcl::Normal>::Ptr & normals)
{
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> estimator;
  estimator.setInputCloud(cloud);
  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  estimator.setSearchMethod(tree);
  estimator.setRadiusSearch(normal_radius_);
  estimator.compute(*normals);
}

void ElevationMappingNode::publishFilteredCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  output.header.stamp = header.stamp;
  output.header.frame_id = target_frame_;
  cloud_pub_->publish(output);
}

bool ElevationMappingNode::getSensorPosition(
  const rclcpp::Time & stamp,
  double & sensor_x,
  double & sensor_y)
{
  if (raytrace_sensor_frame_ == target_frame_) {
    sensor_x = 0.0;
    sensor_y = 0.0;
    return true;
  }

  try {
    const auto transform = tf_buffer_->lookupTransform(
      target_frame_, raytrace_sensor_frame_, stamp, tf2::durationFromSec(0.1));
    sensor_x = transform.transform.translation.x;
    sensor_y = transform.transform.translation.y;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Cannot get raytrace origin from '%s' to '%s': %s",
      raytrace_sensor_frame_.c_str(), target_frame_.c_str(), ex.what());
    return false;
  }
}

bool ElevationMappingNode::worldToGrid(
  double world_x,
  double world_y,
  int & cell_x,
  int & cell_y) const
{
  const double origin_x = -map_size_x_ / 2.0;
  const double origin_y = -map_size_y_ / 2.0;

  cell_x = static_cast<int>(std::floor((world_x - origin_x) / resolution_));
  cell_y = static_cast<int>(std::floor((world_y - origin_y) / resolution_));
  return isInsideGrid(cell_x, cell_y);
}

bool ElevationMappingNode::isInsideGrid(int cell_x, int cell_y) const
{
  return cell_x >= 0 && cell_x < width_ && cell_y >= 0 && cell_y < height_;
}

void ElevationMappingNode::raytraceFreeCells(
  std::vector<uint8_t> & observed_free,
  int start_x,
  int start_y,
  int end_x,
  int end_y) const
{
  int x = start_x;
  int y = start_y;
  const int dx = std::abs(end_x - start_x);
  const int dy = std::abs(end_y - start_y);
  const int step_x = start_x < end_x ? 1 : -1;
  const int step_y = start_y < end_y ? 1 : -1;
  int error = dx - dy;

  std::vector<std::pair<int, int>> ray_cells;
  ray_cells.reserve(static_cast<std::size_t>(std::max(dx, dy) + 1));

  while (true) {
    ray_cells.emplace_back(x, y);
    if (x == end_x && y == end_y) {
      break;
    }

    const int error2 = 2 * error;
    if (error2 > -dy) {
      error -= dy;
      x += step_x;
    }
    if (error2 < dx) {
      error += dx;
      y += step_y;
    }
  }

  const int cells_to_keep = std::max(1, raytrace_endpoint_margin_cells_);
  const int free_cell_count = std::max(
    0, static_cast<int>(ray_cells.size()) - cells_to_keep);

  for (int i = 0; i < free_cell_count; ++i) {
    const auto & cell = ray_cells[static_cast<std::size_t>(i)];
    if (!isInsideGrid(cell.first, cell.second)) {
      continue;
    }

    const std::size_t index = static_cast<std::size_t>(cell.second * width_ + cell.first);
    if (index < observed_free.size()) {
      observed_free[index] = 1U;
    }
  }
}

void ElevationMappingNode::publishTraversabilityGrid(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  const pcl::PointCloud<pcl::Normal>::Ptr & normals,
  const std_msgs::msg::Header & header)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = header.stamp;
  grid.header.frame_id = target_frame_;

  grid.info.resolution = static_cast<float>(resolution_);
  grid.info.width = static_cast<unsigned int>(width_);
  grid.info.height = static_cast<unsigned int>(height_);
  grid.info.origin.position.x = -map_size_x_ / 2.0;
  grid.info.origin.position.y = -map_size_y_ / 2.0;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.x = 0.0;
  grid.info.origin.orientation.y = 0.0;
  grid.info.origin.orientation.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  const std::size_t grid_size =
    static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);

  grid.data.assign(grid_size, -1);
  std::vector<uint8_t> observed_free(grid_size, 0U);
  std::vector<int16_t> observed_cost(grid_size, -1);

  double sensor_x = 0.0;
  double sensor_y = 0.0;
  bool sensor_position_available = false;

  if (raytrace_enabled_) {
    sensor_position_available = getSensorPosition(
      rclcpp::Time(header.stamp), sensor_x, sensor_y);
  }

  int sensor_cell_x = 0;
  int sensor_cell_y = 0;
  if (sensor_position_available) {
    sensor_position_available = worldToGrid(
      sensor_x, sensor_y, sensor_cell_x, sensor_cell_y);

    if (!sensor_position_available) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Raytrace sensor origin is outside the local grid");
    }
  }

  for (std::size_t i = 0; i < cloud->points.size(); ++i) {
    const auto & point = cloud->points[i];
    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    const double dx = static_cast<double>(point.x) - sensor_x;
    const double dy = static_cast<double>(point.y) - sensor_y;
    const double horizontal_range = std::hypot(dx, dy);

    if (horizontal_range < raytrace_min_range_) {
      continue;
    }

    int cell_x = 0;
    int cell_y = 0;
    if (!worldToGrid(point.x, point.y, cell_x, cell_y)) {
      continue;
    }

    if (raytrace_enabled_ &&
        sensor_position_available &&
        horizontal_range <= raytrace_max_range_) {
      raytraceFreeCells(
        observed_free,
        sensor_cell_x,
        sensor_cell_y,
        cell_x,
        cell_y);
    }

    const std::size_t index = static_cast<std::size_t>(cell_y * width_ + cell_x);
    if (index >= grid_size) {
      continue;
    }

    // Keep lethal-height logic so the 360-degree LiDAR endpoint is an obstacle
    // even when its surface normal is invalid.
    const int height_cost = heightToCost(static_cast<double>(point.z));

    int slope_cost = 0;
    if (i < normals->points.size()) {
      const auto & normal = normals->points[i];
      if (std::isfinite(normal.normal_x) &&
          std::isfinite(normal.normal_y) &&
          std::isfinite(normal.normal_z)) {
        const double normal_z = std::clamp(
          std::abs(static_cast<double>(normal.normal_z)), 0.0, 1.0);
        const double slope_deg = std::acos(normal_z) * 180.0 / M_PI;
        slope_cost = slopeToCost(slope_deg);
      }
    }

    const int final_cost = std::clamp(
      std::max(slope_cost, height_cost), 0, 100);

    observed_cost[index] = std::max<int16_t>(
      observed_cost[index], static_cast<int16_t>(final_cost));
  }

  // A measured endpoint always wins over a free ray crossing the same cell.
  for (std::size_t index = 0; index < grid_size; ++index) {
    if (observed_cost[index] >= 0) {
      grid.data[index] = static_cast<int8_t>(
        std::clamp<int>(observed_cost[index], 0, 100));
    } else if (observed_free[index] != 0U) {
      grid.data[index] = 0;
    } else {
      grid.data[index] = -1;
    }
  }

  grid_pub_->publish(grid);
}

int ElevationMappingNode::heightToCost(double point_z) const
{
  if (!std::isfinite(point_z)) {
    return 0;
  }
  if (point_z > obstacle_max_height_) {
    return 0;
  }
  if (point_z < obstacle_min_height_) {
    return 0;
  }
  if (point_z >= lethal_obstacle_height_) {
    return 100;
  }

  const double range = lethal_obstacle_height_ - obstacle_min_height_;
  if (range <= 0.0) {
    return 100;
  }

  const double ratio = (point_z - obstacle_min_height_) / range;
  return static_cast<int>(std::clamp(ratio, 0.0, 1.0) * 70.0);
}

int ElevationMappingNode::slopeToCost(double slope_deg) const
{
  if (!std::isfinite(slope_deg)) {
    return 0;
  }
  if (slope_deg >= hard_slope_deg_) {
    return 100;
  }
  if (slope_deg >= max_slope_deg_) {
    return 70;
  }
  if (max_slope_deg_ <= 0.0) {
    return 100;
  }

  const double ratio = slope_deg / max_slope_deg_;
  return static_cast<int>(std::clamp(ratio, 0.0, 1.0) * 50.0);
}
