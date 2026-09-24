#include <chrono>
#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer (10 Hz)
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!current_path_ || !robot_odom_) {
    return;
  }

  // Edge case: empty path -> stop
  if (current_path_->poses.empty()) {
    stopRobot();
    return;
  }

  // Edge case: close to the final goal -> stop
  const auto &robot_position = robot_odom_->pose.pose.position;
  const auto &final_position = current_path_->poses.back().pose.position;
  if (computeDistance(robot_position, final_position) < goal_tolerance_) {
    stopRobot();
    return;
  }

  // Find the lookahead point
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    return;  // No valid lookahead point found
  }

  // Compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}

// the first point on the path that is at least lookahead_distance_ away from the robot
std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  const auto &robot_position = robot_odom_->pose.pose.position;

  for (size_t i = 0; i < current_path_->poses.size(); ++i) {
    double distance = computeDistance(robot_position, current_path_->poses[i].pose.position);
    if (distance >= lookahead_distance_) {
      return current_path_->poses[i];
    }
  }

  // every point is closer than the lookahead distance (we are near the end), so aim at the last one
  return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd_vel;

  const auto &robot_position = robot_odom_->pose.pose.position;
  double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

  // direction from the robot to the lookahead point
  double dx = target.pose.position.x - robot_position.x;
  double dy = target.pose.position.y - robot_position.y;
  double angle_to_target = std::atan2(dy, dx);

  // steering angle: how far the target is from where the robot is facing
  double alpha = angle_to_target - robot_yaw;

  // keep alpha between -pi and pi
  while (alpha > M_PI) {
    alpha -= 2.0 * M_PI;
  }
  while (alpha < -M_PI) {
    alpha += 2.0 * M_PI;
  }

  // curvature of the circular arc from the robot to the lookahead point
  double distance = std::sqrt(dx * dx + dy * dy);
  double curvature = 2.0 * std::sin(alpha) / distance;

  // constant linear speed, angular speed follows the curvature
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = linear_speed_ * curvature;

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

// quaternion -> yaw (the robot's heading on the flat ground)
double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

// send one zero velocity command, then forget the path so the loop goes quiet
void ControlNode::stopRobot() {
  geometry_msgs::msg::Twist stop;   // all zeros
  cmd_vel_pub_->publish(stop);
  current_path_.reset();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
