#include <cmath>
#include <memory>
#include <vector>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(//create subscription means on topic lidar, expect laserscan tpye message
      "/lidar", 10,
      std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}
void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Fresh grid every scan. 0 means "nothing here, go ahead"
  int total_cells = width_ * height_;
  std::vector<int8_t> grid(total_cells, 0);

  for (size_t i = 0; i < scan->ranges.size(); ++i) {

    // How far this one laser beam travelled before it hit something
    double range = scan->ranges[i];

    // Too close or too far means the sensor didn't really see anything
    if (range < scan->range_min) {
      continue;
    }
    if (range > scan->range_max) {
      continue;
    }

    // Which direction was this beam pointing?
    double angle = scan->angle_min + i * scan->angle_increment;

    // Turn "2.3 m at 30 degrees" into "x metres across, y metres up"
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    // Now turn metres into a square on the graph paper
    double x_from_corner = x - origin_x_;
    double y_from_corner = y - origin_y_;

    int gx = x_from_corner / resolution_;
    int gy = y_from_corner / resolution_;

    // Anything past the edge of our grid gets ignored
    if (gx < 0 || gx >= width_) {
      continue;
    }
    if (gy < 0 || gy >= height_) {
      continue;
    }

    // The grid is one long list, so skip gy whole rows then walk gx across
    int index = gy * width_ + gx;
    grid[index] = 100;
  }
    // Put a danger zone around every wall so we don't clip them
  inflateObstacles(grid);

  // Package it up the way ROS expects
  nav_msgs::msg::OccupancyGrid msg;

  msg.header.stamp = scan->header.stamp;
  msg.header.frame_id = scan->header.frame_id;

  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;

  msg.data = grid;

  costmap_pub_->publish(msg);
}
void CostmapNode::inflateObstacles(std::vector<int8_t>& grid) {
  // Copy the walls first, otherwise the halo spreads onto itself forever
  std::vector<int8_t> walls = grid;

  int radius_in_cells = inflation_radius_ / resolution_;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {

      int index = y * width_ + x;

      // Only walls get a halo
      if (walls[index] != 100) {
        continue;
      }

      // Look at every cell in a square around this wall
      for (int dy = -radius_in_cells; dy <= radius_in_cells; ++dy) {
        for (int dx = -radius_in_cells; dx <= radius_in_cells; ++dx) {

          int nx = x + dx;
          int ny = y + dy;

          if (nx < 0 || nx >= width_) {
            continue;
          }
          if (ny < 0 || ny >= height_) {
            continue;
          }

          // A square has corners, so check the real distance to keep it circular
          double dist = std::sqrt(dx * dx + dy * dy) * resolution_;

          if (dist > inflation_radius_) {
            continue;
          }

          // Closer to the wall means scarier. 100 at the wall, 0 at the edge
          int cost = 100 * (1.0 - dist / inflation_radius_);

          int neighbour_index = ny * width_ + nx;

          // If two walls overlap, keep the scarier number
          if (cost > grid[neighbour_index]) {
            grid[neighbour_index] = cost;
          }
        }
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
