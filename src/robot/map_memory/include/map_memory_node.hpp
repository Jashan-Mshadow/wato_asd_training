#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

// same idea as the costmap header: this is just the list of what this node owns
class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr new_costmap); // runs every time the costmap node publishes
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);                // runs every time we get a new robot position
    void publishMap();                                                               // runs on a timer, once a second
    void pasteCostmapIntoMap();                                                      // the actual stitching

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // the big map that never forgets
    std::vector<int8_t> big_map_;

    // the newest costmap from node 1, saved so the timer can use it
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool have_costmap_ = false;

    // where the robot is right now, in world coordinates
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_heading_ = 0.0;   // radians, 0 means facing along +x

    // where the robot was the last time we pasted something in
    double last_paste_x_ = 0.0;
    double last_paste_y_ = 0.0;
    bool pasted_once_ = false;

    // map settings (same size and resolution as the costmap, keeps the math simple)
    static constexpr double resolution_ = 0.1;   // metres per cell
    static constexpr int    width_  = 300;       // 30 metres across
    static constexpr int    height_ = 300;
    static constexpr double origin_x_ = -15.0;
    static constexpr double origin_y_ = -15.0;

    // only bother updating the map once the robot has actually gone somewhere
    static constexpr double distance_between_updates_ = 1.5;  // metres
};

#endif
