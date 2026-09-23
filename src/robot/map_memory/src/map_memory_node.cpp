#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {

  // listen to node 1's costmap
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  // listen to where the robot is
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // the big map starts out completely empty
  int total_cells = width_ * height_;
  big_map_.assign(total_cells, 0);

  // publish once a second, even if nothing changed, so the planner always has a map
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&MapMemoryNode::publishMap, this));
}

// just hang on to the newest costmap, the timer decides when to use it
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr new_costmap) {
  latest_costmap_ = *new_costmap;
  have_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {

  robot_x_ = odom->pose.pose.position.x;
  robot_y_ = odom->pose.pose.position.y;

  // the heading is stored as a quaternion (4 numbers), we only care about the flat spin
  double qx = odom->pose.pose.orientation.x;
  double qy = odom->pose.pose.orientation.y;
  double qz = odom->pose.pose.orientation.z;
  double qw = odom->pose.pose.orientation.w;

  double top = 2.0 * (qw * qz + qx * qy);
  double bottom = 1.0 - 2.0 * (qy * qy + qz * qz);

  robot_heading_ = std::atan2(top, bottom);
}

void MapMemoryNode::publishMap() {

  // nothing from node 1 yet, so just send the empty map
  if (have_costmap_) {

    // how far have we moved since the last time we pasted something in?
    double dx = robot_x_ - last_paste_x_;
    double dy = robot_y_ - last_paste_y_;
    double distance_moved = std::sqrt(dx * dx + dy * dy);

    // first time through we always paste, after that only once we've actually driven somewhere
    bool far_enough = distance_moved >= distance_between_updates_;

    if (!pasted_once_ || far_enough) {
      pasteCostmapIntoMap();

      last_paste_x_ = robot_x_;
      last_paste_y_ = robot_y_;
      pasted_once_ = true;
    }
  }

  nav_msgs::msg::OccupancyGrid msg;

  msg.header.stamp = this->now();
  msg.header.frame_id = "sim_world";   // this map is in world coordinates, not robot coordinates

  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;

  msg.data = big_map_;

  map_pub_->publish(msg);
}

// take the costmap (drawn from the robot's point of view) and stamp it onto the big world map
void MapMemoryNode::pasteCostmapIntoMap() {

  int costmap_width = latest_costmap_.info.width;
  int costmap_height = latest_costmap_.info.height;
  double costmap_resolution = latest_costmap_.info.resolution;
  double costmap_origin_x = latest_costmap_.info.origin.position.x;
  double costmap_origin_y = latest_costmap_.info.origin.position.y;

  // precompute these, they are the same for every cell
  double cos_heading = std::cos(robot_heading_);
  double sin_heading = std::sin(robot_heading_);

  for (int y = 0; y < costmap_height; ++y) {
    for (int x = 0; x < costmap_width; ++x) {

      int costmap_index = y * costmap_width + x;
      int8_t cell_value = latest_costmap_.data[costmap_index];

      // empty cells carry no information, skip them so we do not erase walls we already know about
      if (cell_value <= 0) {
        continue;
      }

      // where is this cell, in metres, relative to the robot?
      double local_x = x * costmap_resolution + costmap_origin_x;
      double local_y = y * costmap_resolution + costmap_origin_y;

      // turn it by the robot's heading, then slide it over to the robot's position
      double world_x = robot_x_ + (local_x * cos_heading - local_y * sin_heading);
      double world_y = robot_y_ + (local_x * sin_heading + local_y * cos_heading);

      // metres to a square on the big map
      int map_x = (world_x - origin_x_) / resolution_;
      int map_y = (world_y - origin_y_) / resolution_;

      if (map_x < 0 || map_x >= width_) {
        continue;
      }
      if (map_y < 0 || map_y >= height_) {
        continue;
      }

      int map_index = map_y * width_ + map_x;

      // if we already saw something scarier here, keep the scarier number
      if (cell_value > big_map_[map_index]) {
        big_map_[map_index] = cell_value;
      }
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
