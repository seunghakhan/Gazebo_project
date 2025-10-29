#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <cmath>
#include <map>

using std::placeholders::_1;

class PID {
public:
  PID(double Kp, double Ki, double Kd)
  : Kp_(Kp), Ki_(Ki), Kd_(Kd), prev_error_(0.0), integral_(0.0) {}
  double compute(double error, double dt) {
    integral_ += error * dt;
    double derivative = dt > 0.0 ? (error - prev_error_) / dt : 0.0;
    double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
    prev_error_ = error;
    return output;
  }
  void reset() { prev_error_ = 0.0; integral_ = 0.0; }
private:
  double Kp_, Ki_, Kd_, prev_error_, integral_;
};

class PathFollower : public rclcpp::Node {
public:
  PathFollower()
  : Node("path_follower"), pose_received_(false), current_target_index_(0),
    pid_speed_m1_(0.7, 0.0, 0.05), pid_speed_other_(0.8, 0.0, 0.05),
    pid_steer_(3.5, 0.0, 0.25), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
    current_speed_(0.0), prev_goal_state_(-1)
  {
    declare_parameter<std::string>("path_file", "path.csv");
    declare_parameter<double>("acceleration_limit", 1.5);
    declare_parameter<double>("speed_state_1", 2.0);
    declare_parameter<double>("speed_state_2", 3.0);
    declare_parameter<double>("speed_state_3", 3.5);
    declare_parameter<double>("speed_state_4", 0.0);

    get_parameter("path_file", path_file_);
    get_parameter("acceleration_limit", acceleration_limit_);
    get_parameter("speed_state_1", state_speeds_[1]);
    get_parameter("speed_state_2", state_speeds_[2]);
    get_parameter("speed_state_3", state_speeds_[3]);
    get_parameter("speed_state_4", state_speeds_[4]);

    state_arrival_thresholds_ = {{1, 0.5}, {2, 2.5}, {3, 3.5}, {4, 0.5}};

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/model/X1_asp/cmd_vel", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);
    drone_pub_ = create_publisher<std_msgs::msg::String>("/drone/command", 10);

    load_path();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PathFollower::control_loop, this));
  }

private:
  void load_path() {
    std::ifstream file(path_file_);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open path file: %s", path_file_.c_str());
      return;
    }
    std::string line;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string x_str, y_str, state_str;
      if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',') && std::getline(ss, state_str)) {
        double x = std::stod(x_str);
        double y = std::stod(y_str);
        int state = std::stoi(state_str);
        if (state <= 4) path_.emplace_back(x, y, state);
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu path points.", path_.size());
  }

  bool update_current_pose() {
    try {
      auto transform = tf_buffer_.lookupTransform("map", "X1_asp/base_link", tf2::TimePointZero);
      current_x_ = transform.transform.translation.x;
      current_y_ = transform.transform.translation.y;
      tf2::Quaternion q;
      tf2::fromMsg(transform.transform.rotation, q);
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);
      pose_received_ = true;
      return true;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "[TF FAIL] %s", ex.what());
      return false;
    }
  }

  void control_loop() {
    if (!update_current_pose() || path_.empty()) return;
    if (current_target_index_ >= path_.size()) return;

    auto [goal_x, goal_y, goal_state] = path_[current_target_index_];
    if (goal_state != prev_goal_state_) {
      pid_speed_m1_.reset(); pid_speed_other_.reset();
      prev_goal_state_ = goal_state;
    }

    double dx = goal_x - current_x_;
    double dy = goal_y - current_y_;
    double dist = std::hypot(dx, dy);
    double ang_err = std::atan2(std::sin(std::atan2(dy, dx) - current_yaw_), std::cos(std::atan2(dy, dx) - current_yaw_));

    if (dist < state_arrival_thresholds_[goal_state]) {
      RCLCPP_INFO(get_logger(), "Reached waypoint %zu (state %d) at (x=%.2f, y=%.2f)", current_target_index_, goal_state, goal_x, goal_y);

      if (goal_state == 1 || goal_state == 4) {
        std_msgs::msg::String msg;
        msg.data = (goal_state == 1) ? "takeoff_mission1" : "land_mission3";
        drone_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "📡 Published drone command: %s", msg.data.c_str());

        cmd_pub_->publish(geometry_msgs::msg::Twist());
        current_speed_ = 0.0;

        if (goal_state == 1) rclcpp::sleep_for(std::chrono::seconds(3));
        if (goal_state == 4) return;
      }

      ++current_target_index_;
      return;
    }

    double dt = 0.1;
    double slowdown_ratio = 1.0;
    if (goal_state == 1 || goal_state == 4) {
      double slowdown_range = (goal_state == 1) ? 6.0 : 0.5;
      slowdown_ratio = (dist < slowdown_range) ? (dist / slowdown_range) : 1.0;
    }

    double target_speed = state_speeds_[goal_state];
    double desired_speed = (goal_state == 1)
      ? pid_speed_m1_.compute(dist, dt)
      : pid_speed_other_.compute(dist, dt);
    desired_speed = std::clamp(desired_speed * slowdown_ratio, 0.0, target_speed);

    if ((goal_state == 2 || goal_state == 3) && desired_speed < 1.5)
      desired_speed = 1.5;

    double max_delta = ((goal_state == 1) ? acceleration_limit_ : acceleration_limit_ * 1.5) * dt;
    current_speed_ += std::clamp(desired_speed - current_speed_, -max_delta, max_delta);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = current_speed_;
    cmd.angular.z = std::clamp(pid_steer_.compute(ang_err, dt), -1.0, 1.0);
    cmd_pub_->publish(cmd);

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now(); marker.header.frame_id = "map";
    marker.ns = "target"; marker.id = 0; marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = goal_x; marker.pose.position.y = goal_y; marker.pose.position.z = 0.2;
    marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3;
    marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_pub_->publish(marker);
  }

  std::string path_file_;
  double acceleration_limit_, current_x_{0.0}, current_y_{0.0}, current_yaw_{0.0}, current_speed_;
  int prev_goal_state_;
  bool pose_received_;
  size_t current_target_index_;
  std::vector<std::tuple<double, double, int>> path_;
  std::map<int, double> state_speeds_, state_arrival_thresholds_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drone_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  PID pid_speed_m1_, pid_speed_other_, pid_steer_;
  tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
  return 0;
}
