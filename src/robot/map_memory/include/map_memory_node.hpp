#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

// Map memory node: stitches the robot-centred costmaps into one big map of the world
class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    // Callbacks
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Timer-based map update
    void updateMap();

    // Integrate the latest costmap into the global map
    void integrateCostmap();

    robot::MapMemoryCore map_memory_;

    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Global map and robot position
    nav_msgs::msg::OccupancyGrid global_map_;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    static constexpr double distance_threshold_ = 1.5;   // metres

    // Latest costmap
    nav_msgs::msg::OccupancyGrid latest_costmap_;

    // Flags
    bool costmap_updated_ = false;
    bool should_update_map_ = true;   // true at startup so a map gets published on initialization
    bool odom_received_ = false;      // don't stitch until we know where the robot is

    // Global map settings. The costmap is 0.1 m per cell, so the map is coarser (0.2 m per cell).
    // The wiki says the costmap resolution should be finer than the map's, so no holes are left.
    static constexpr double resolution_ = 0.2;   // metres per cell
    static constexpr int    width_  = 150;       // 150 cells * 0.2 m = 30 m, the whole world
    static constexpr int    height_ = 150;
    static constexpr double origin_x_ = -15.0;   // the world's walls are at -15 and +15
    static constexpr double origin_y_ = -15.0;
};

#endif
