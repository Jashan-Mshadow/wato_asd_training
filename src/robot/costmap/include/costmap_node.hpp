#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

// Costmap node: turns each lidar scan into a grid of "how dangerous is this square"
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

  private:
    // runs every time a new LaserScan arrives on /lidar
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // the 5 steps from the wiki
    void initializeCostmap();
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap(const std_msgs::msg::Header &scan_header);

    robot::CostmapCore costmap_;

    // 1 subscriber (/lidar) and 1 publisher (/costmap)
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    // the 2D costmap array, used as grid_[y][x]
    std::vector<std::vector<int8_t>> grid_;

    // grid settings
    static constexpr double resolution_ = 0.1;       // metres per cell
    static constexpr int    width_  = 300;           // 300 cells * 0.1 m = 30 m
    static constexpr int    height_ = 300;
    static constexpr double origin_x_ = -15.0;       // cell (0,0) is 15 m behind and 15 m right of the robot,
    static constexpr double origin_y_ = -15.0;       // so the robot sits in the middle of the grid

    // inflation settings
    static constexpr double inflation_radius_ = 1.5; // metres (1.0 let the 1 m wide robot scrape box corners)
    static constexpr int    max_cost_ = 100;         // cost of an actual obstacle
};

#endif
