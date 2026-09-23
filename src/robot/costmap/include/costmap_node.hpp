#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

//this files bascally is a list of everyhting that ecist and is gonna be used
//side note, im used o python so im used to using # instead of //, kep doing it by accident :()
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

  private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan); //laserscan is what im receiving, lidarcallback is the function that is run everytime we get a message
    void inflateObstacles(std::vector<int8_t>& grid); 

    robot::CostmapCore costmap_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    // Grid settings
    static constexpr double resolution_ = 0.1;    // metres per cell
    static constexpr int    width_  = 300;        // 300 cells = 30 metres
    static constexpr int    height_ = 300;
    static constexpr double origin_x_ = -15.0;    // where cell (0,0) sits in the world
    static constexpr double origin_y_ = -15.0;
    static constexpr double inflation_radius_ = 1.0;
};

#endif