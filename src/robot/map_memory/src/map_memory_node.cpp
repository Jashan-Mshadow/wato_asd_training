#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Initialize subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer (every 1 second)
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  // Set up the global map (in world coordinates)
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = resolution_;
  global_map_.info.width = width_;
  global_map_.info.height = height_;
  global_map_.info.origin.position.x = origin_x_;
  global_map_.info.origin.position.y = origin_y_;
  global_map_.info.origin.orientation.w = 1.0;

  // every cell starts at 0 (free), same default as the costmap
  for (int i = 0; i < width_ * height_; ++i) {
    global_map_.data.push_back(0);
  }
}

// Callback for costmap updates
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Store the latest costmap
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

// Callback for odometry updates
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // quaternion -> yaw (the robot's heading on the flat ground)
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  robot_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  odom_received_ = true;

  // Compute distance traveled
  double distance = std::sqrt(std::pow(robot_x_ - last_x_, 2) + std::pow(robot_y_ - last_y_, 2));
  if (distance >= distance_threshold_) {
    last_x_ = robot_x_;
    last_y_ = robot_y_;
    should_update_map_ = true;
  }
}

// Timer-based map update
void MapMemoryNode::updateMap() {
  // before the first odometry message the robot's position is unknown (it would default to 0,0),
  // so stitching then would put the whole costmap in the wrong place
  if (should_update_map_ && costmap_updated_ && odom_received_) {
    integrateCostmap();
    should_update_map_ = false;
  }

  // publish every tick, not only after an update: if the planner missed a message
  // (e.g. it was still starting up) it gets the map again a second later
  global_map_.header.stamp = this->now();
  map_pub_->publish(global_map_);
}

// Integrate the latest costmap into the global map
void MapMemoryNode::integrateCostmap() {
  int costmap_width = latest_costmap_.info.width;
  int costmap_height = latest_costmap_.info.height;
  double costmap_resolution = latest_costmap_.info.resolution;
  double costmap_origin_x = latest_costmap_.info.origin.position.x;
  double costmap_origin_y = latest_costmap_.info.origin.position.y;

  for (int y = 0; y < costmap_height; ++y) {
    for (int x = 0; x < costmap_width; ++x) {

      int8_t value = latest_costmap_.data[y * costmap_width + x];

      // If a cell in the new costmap is unknown (-1), retain the previous value in the global map
      if (value < 0) {
        continue;
      }

      // centre of this costmap cell, in metres, relative to the robot
      double local_x = costmap_origin_x + (x + 0.5) * costmap_resolution;
      double local_y = costmap_origin_y + (y + 0.5) * costmap_resolution;

      // Transform into the global frame: rotate by the robot's yaw, then move to the robot's position
      double world_x = robot_x_ + local_x * std::cos(robot_yaw_) - local_y * std::sin(robot_yaw_);
      double world_y = robot_y_ + local_x * std::sin(robot_yaw_) + local_y * std::cos(robot_yaw_);

      // metres -> global map cell
      int map_x = static_cast<int>(std::floor((world_x - origin_x_) / resolution_));
      int map_y = static_cast<int>(std::floor((world_y - origin_y_) / resolution_));

      // parts of the costmap that land outside the global map are ignored
      if (map_x < 0 || map_x >= width_) {
        continue;
      }
      if (map_y < 0 || map_y >= height_) {
        continue;
      }

      // Known value (occupied or free): overwrite, new data wins over old data
      global_map_.data[map_y * width_ + map_x] = value;
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
