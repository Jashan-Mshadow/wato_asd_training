#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;

  // the map updated, so replan
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  goal_start_time_ = this->now();
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {

    // goal reached -> back to waiting for a goal
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
      return;
    }

    // timeout -> give up, back to waiting for a goal
    double seconds_since_goal = (this->now() - goal_start_time_).seconds();
    if (seconds_since_goal > timeout_seconds_) {
      RCLCPP_WARN(this->get_logger(), "Timed out, giving up on this goal");
      state_ = State::WAITING_FOR_GOAL;

      // an empty path tells the control node to stop
      nav_msgs::msg::Path empty_path;
      empty_path.header.stamp = this->now();
      empty_path.header.frame_id = current_map_.header.frame_id;
      path_pub_->publish(empty_path);
      return;
    }

    // otherwise keep replanning from where the robot is now
    RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
    planPath();
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < goal_threshold_;
}

void PlannerNode::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }

  double resolution = current_map_.info.resolution;
  double origin_x = current_map_.info.origin.position.x;
  double origin_y = current_map_.info.origin.position.y;

  // Initialize start and goal points based on the robot's position and the goal point
  CellIndex start(static_cast<int>(std::floor((robot_pose_.position.x - origin_x) / resolution)),
                  static_cast<int>(std::floor((robot_pose_.position.y - origin_y) / resolution)));
  CellIndex goal(static_cast<int>(std::floor((goal_.point.x - origin_x) / resolution)),
                 static_cast<int>(std::floor((goal_.point.y - origin_y) / resolution)));

  // A* on the current map
  std::vector<CellIndex> cells = aStar(start, goal);

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = current_map_.header.frame_id;   // "sim_world", same frame as the map

  // Fill path.poses with the resulting waypoints (the middle of each cell)
  for (size_t i = 0; i < cells.size(); ++i) {
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header = path.header;
    waypoint.pose.position.x = origin_x + (cells[i].x + 0.5) * resolution;
    waypoint.pose.position.y = origin_y + (cells[i].y + 0.5) * resolution;
    waypoint.pose.orientation.w = 1.0;
    path.poses.push_back(waypoint);
  }

  if (cells.empty()) {
    RCLCPP_WARN(this->get_logger(), "No valid path exists");
  }

  path_pub_->publish(path);
}

std::vector<CellIndex> PlannerNode::aStar(CellIndex start, CellIndex goal) {
  int width = current_map_.info.width;
  int height = current_map_.info.height;

  std::vector<CellIndex> path;

  // start or goal off the map means there is no path
  if (start.x < 0 || start.x >= width || start.y < 0 || start.y >= height) {
    return path;
  }
  if (goal.x < 0 || goal.x >= width || goal.y < 0 || goal.y >= height) {
    return path;
  }

  // open list: nodes to be evaluated, smallest f on top
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_list;

  // closed list: nodes already evaluated
  std::unordered_map<CellIndex, bool, CellIndexHash> closed_list;

  // g(n): cost to reach each node from the start
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  // which node we came from, to rebuild the path at the end
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  g_score[start] = 0.0;
  open_list.push(AStarNode(start, 0.0));

  bool found_goal = false;

  // Expand nodes until the goal is reached or no valid path exists
  while (!open_list.empty()) {

    // take the node with the smallest f
    CellIndex current = open_list.top().index;
    open_list.pop();

    if (current == goal) {
      found_goal = true;
      break;
    }

    // a node can be in the open list more than once, only evaluate it the first time
    if (closed_list[current]) {
      continue;
    }
    closed_list[current] = true;

    // look at the 8 neighbours
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {

        if (dx == 0 && dy == 0) {
          continue;
        }

        CellIndex neighbour(current.x + dx, current.y + dy);

        if (neighbour.x < 0 || neighbour.x >= width) {
          continue;
        }
        if (neighbour.y < 0 || neighbour.y >= height) {
          continue;
        }
        if (closed_list[neighbour]) {
          continue;
        }

        int cell_cost = current_map_.data[neighbour.y * width + neighbour.x];

        // obstacle constraint: never go into an obstacle
        if (cell_cost >= obstacle_cost_) {
          continue;
        }

        // unknown cells (-1) count as free
        if (cell_cost < 0) {
          cell_cost = 0;
        }

        // distance of this step: 1 straight, about 1.414 diagonal
        double step = std::sqrt(dx * dx + dy * dy);

        // g(n) = cost so far + step + the cell's cost, so the path minimizes the cost
        double new_g = g_score[current] + step + cell_cost;

        // skip if we already know a cheaper way to this neighbour
        if (g_score.count(neighbour) > 0 && new_g >= g_score[neighbour]) {
          continue;
        }

        g_score[neighbour] = new_g;
        came_from[neighbour] = current;

        // h(n): Euclidean distance to the goal
        double hx = goal.x - neighbour.x;
        double hy = goal.y - neighbour.y;
        double h = std::sqrt(hx * hx + hy * hy);

        // f(n) = g(n) + h(n)
        open_list.push(AStarNode(neighbour, new_g + h));
      }
    }
  }

  if (!found_goal) {
    return path;   // empty: no valid path
  }

  // walk back from the goal to the start
  CellIndex current = goal;
  path.push_back(current);
  while (current != start) {
    current = came_from[current];
    path.push_back(current);
  }

  // it was built backwards, so flip it
  std::reverse(path.begin(), path.end());

  return path;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
