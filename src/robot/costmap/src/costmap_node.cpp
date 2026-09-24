#include <cmath>
#include <memory>
#include <vector>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Subscriber: LaserScan messages from /lidar
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // Publisher: OccupancyGrid messages to /costmap
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Step 1: Initialize costmap
  initializeCostmap();

  // Step 2: Convert LaserScan to grid and mark obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min) {
      // Calculate grid coordinates
      int x_grid;
      int y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }

  // Step 3: Inflate obstacles
  inflateObstacles();

  // Step 4: Publish costmap
  publishCostmap(scan->header);
}

// Make a fresh 2D array and set every cell to 0 (free space)
void CostmapNode::initializeCostmap() {
  grid_.clear();

  for (int y = 0; y < height_; ++y) {
    std::vector<int8_t> row(width_, 0);
    grid_.push_back(row);
  }
}

// Polar (range, angle) -> Cartesian (x, y) metres -> grid indices
void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
  // x = range * cos(angle), y = range * sin(angle)
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  // shift so cell (0,0) is the corner of the grid, then divide by the cell size
  x_grid = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  y_grid = static_cast<int>(std::floor((y - origin_y_) / resolution_));
}

// Set the obstacle's cell to the high cost (100 = occupied)
void CostmapNode::markObstacle(int x_grid, int y_grid) {
  // hits outside the grid are ignored
  if (x_grid < 0 || x_grid >= width_) {
    return;
  }
  if (y_grid < 0 || y_grid >= height_) {
    return;
  }

  grid_[y_grid][x_grid] = max_cost_;
}

// Give every cell near an obstacle a cost that fades out linearly with distance
void CostmapNode::inflateObstacles() {
  // copy of the grid before inflating, so we only inflate around real obstacles
  std::vector<std::vector<int8_t>> obstacles = grid_;

  int radius_in_cells = static_cast<int>(inflation_radius_ / resolution_);

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {

      // For each obstacle cell
      if (obstacles[y][x] != max_cost_) {
        continue;
      }

      // look at the cells around it
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

          // Calculate the Euclidean distance to this cell (in metres)
          double distance = std::sqrt(dx * dx + dy * dy) * resolution_;

          // Do not assign a cost to cells beyond the inflation radius
          if (distance > inflation_radius_) {
            continue;
          }

          // cost = max_cost * (1 - distance / inflation_radius)
          int cost = static_cast<int>(max_cost_ * (1.0 - distance / inflation_radius_));

          // Only assign a cost if it is higher than the cell's current value
          if (cost > grid_[ny][nx]) {
            grid_[ny][nx] = cost;
          }
        }
      }
    }
  }
}

// Convert the 2D array into an OccupancyGrid message and publish it on /costmap
void CostmapNode::publishCostmap(const std_msgs::msg::Header &scan_header) {
  nav_msgs::msg::OccupancyGrid msg;

  // header: frame ID and timestamp
  msg.header.stamp = scan_header.stamp;
  msg.header.frame_id = scan_header.frame_id;   // the lidar's frame, the grid moves with the robot

  // info: resolution, origin and size
  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;

  // data: flatten the 2D array into a 1D array, one row after another
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      msg.data.push_back(grid_[y][x]);
    }
  }

  costmap_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
