#include <chrono>
#include <functional>
#include <memory>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class TurtleBotController : public rclcpp::Node {

private:

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;

  // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
  tf2::Quaternion tf_quat;
  
  // PID Gain
  double Kpel = 0.3, Kdel = 0.1, Kiel = 0.01;
  double Kpet = 2.0, Kdet = 1.0, Kiet = 0.02;

  // Desired position
  double desired_x_;
  double desired_y_;

  // List of waypoints (desired positions)
  std::vector<std::pair<double, double>> waypoints_;
  size_t current_waypoint_;

  // PID errors
  double error_el_;
  double error_et_;
  double integral_el_;
  double integral_et_;
  double previous_error_el_;
  double previous_error_et_;

  double vmax_ = 1.0;

  double lookaheadDist_ = 0.2;

  double dt = 0.1;  // Time step (0.1 seconds)

  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr pose) {

    // Calculate the errors
    double error_x_ =  desired_x_ - pose->pose.pose.position.x ;
    double error_y_ =  desired_y_ - pose->pose.pose.position.y ;
    double distance_error = sqrt(pow(error_x_,2) + pow(error_y_,2));
    double theta_error = atan2(error_y_ ,error_x_);

    const geometry_msgs::msg::Quaternion& quat = pose->pose.pose.orientation;
    // Convert the quaternion to Euler angles (roll, pitch, yaw)
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    error_el_ = distance_error ;
    error_et_ = theta_error -yaw ;

    while (error_et_ < -M_PI)
        error_et_ += 2.0 * M_PI;
    while (error_et_ > M_PI)
        error_et_ -= 2.0 * M_PI;

    // Calculate the integrals
    integral_el_ += error_el_ * dt;
    integral_et_ += error_et_ * dt;

    // Calculate the derivatives
    double derivative_el_ = ( error_el_ - previous_error_el_) / dt;
    double derivative_et_ = (error_et_ - previous_error_et_) ;

    while (derivative_et_ < -M_PI)
        derivative_et_ += 2.0 * M_PI;
    while (derivative_et_ > M_PI)
        derivative_et_  -= 2.0 * M_PI;

    // Calculate the control signals using PID equation
    double control_signal_v = Kpel * error_el_ + Kdel * derivative_el_  + Kiel * integral_el_;
    double control_signal_w = Kpet * error_et_ + Kdet * (derivative_et_ /dt) + Kiet * integral_et_ ;
    
    while (control_signal_v > vmax_)
      control_signal_v = vmax_;
    while (control_signal_v < -vmax_)
      control_signal_v = -vmax_;
    
    // Create the Twist message for publishing velocity commands
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = control_signal_v;
    cmd_vel.angular.z = control_signal_w;

    std::cout << "Input linear v :"  << control_signal_v << "angular u: " << control_signal_w << std::endl;

    if (distance_error <= lookaheadDist_)
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      publisher_->publish(cmd_vel);
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", current_waypoint_);

      // Check if there are more waypoints
      if (current_waypoint_ < waypoints_.size()) {
        // Move to the next waypoint
        desired_x_ = waypoints_[current_waypoint_].first;
        desired_y_ = waypoints_[current_waypoint_].second;
        current_waypoint_++;
      } else {
        RCLCPP_INFO(this->get_logger(), "All waypoints reached!");
        // Stop the robot
        desired_x_ = pose->pose.pose.position.x;
        desired_y_ = pose->pose.pose.position.y;
      }
    }

    // Publish the velocity command
    publisher_->publish(cmd_vel);

    // Update previous errors
    previous_error_el_ = error_el_;
    previous_error_et_ = error_et_;

  }

public:
  TurtleBotController() : Node("path_follower") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&TurtleBotController::poseCallback, this, std::placeholders::_1));

    // Initialize PID errors
    error_el_ = 0.0;
    error_et_ = 0.0;
    integral_el_ = 0.0;
    integral_et_ = 0.0;
    previous_error_el_ = 0.0;
    previous_error_et_ = 0.0;

    // Initialize waypoints
    waypoints_ = {{-5.0, 3.0}, {3.0, 5.0}, {0.0, 0.0}}; // Add more waypoints as needed
    current_waypoint_ = 0;

    // Set the initial desired position
    desired_x_ = waypoints_[current_waypoint_].first;
    desired_y_ = waypoints_[current_waypoint_].second;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleBotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}