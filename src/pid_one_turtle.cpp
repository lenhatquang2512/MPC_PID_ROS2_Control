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
  rclcpp::TimerBase::SharedPtr timer_;

  // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
  tf2::Quaternion tf_quat;
  
  // PID Gain
  double Kpel = 1.0, Kdel = 0.01, Kiel = 0.01;
  double Kpet = 0.5, Kdet = 0.01, Kiet = 0.01;

  // Desired position
  double desired_x_;
  double desired_y_;

  // PID errors
  double error_el_;
  double error_et_;
  double integral_el_;
  double integral_et_;
  double previous_error_el_;
  double previous_error_et_;

  double vmax_ = 1.0;
  double Kvet_ = 1.0, Kvel_ = 1.0;

  double lookaheadDist_ = 0.2;

  double dt = 0.01;  // Time step (0.1 seconds)
  
  /**
   * @brief 
   * 
   * @param pose 
   */
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr pose) {

    double desired_theta = atan2(desired_y_,desired_x_);

    // Calculate the errors
    double error_x_ =  pose->pose.pose.position.x - desired_x_ ;
    double error_y_ =  pose->pose.pose.position.y - desired_y_ ;

    double yy = error_x_ * sin(-desired_theta) + error_y_ * cos(-desired_theta);
    double desired_dis = sqrt(pow(error_x_,2) + pow(error_y_,2));

    const geometry_msgs::msg::Quaternion& quat = pose->pose.pose.orientation;

    // Convert the quaternion to Euler angles (roll, pitch, yaw)
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    error_el_ = -yy;
    // error_el_ = desired_dis;
    error_et_ = desired_theta - yaw;

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
    double control_signal_v = vmax_ - Kvel_ * fabs(error_el_) - Kvet_ * fabs(error_et_);
    // double control_signal_v = Kpel * error_el_ + Kdel * derivative_el_  + Kiel * integral_el_;
    double control_signal_w =  Kpet * error_et_ + Kdet * (derivative_et_ /dt) + Kiet * integral_et_ ;
    control_signal_w += Kpel * error_el_ + Kdel * derivative_el_  + Kiel * integral_el_;

    // Create the Twist message for publishing velocity commands
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = control_signal_v;
    cmd_vel.angular.z = control_signal_w;

    if (desired_dis <= lookaheadDist_)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        publisher_->publish(cmd_vel);
    }

     // Publish the velocity command
    publisher_->publish(cmd_vel);

    // Update previous errors
    previous_error_el_ = error_el_;
    previous_error_et_ = error_et_;
    
  }

    void controlLoop() {
    // This function is called periodically for control loop updates
    // You can add any additional logic here
  }

public:
  /**
   * @brief Construct a new Turtle Bot Controller object
   * 
   */
  TurtleBotController() : Node("turtlebot_controller") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&TurtleBotController::poseCallback, this, std::placeholders::_1));


    // Set the desired position
    desired_x_ = 2.34;
    desired_y_ = 3.5;

    // Initialize PID errors
    error_el_ = 0.0;
    error_et_ = 0.0;
    integral_el_ = 0.0;
    integral_et_ = 0.0;
    previous_error_el_ = 0.0;
    previous_error_et_ = 0.0;

    // Control loop timer
    timer_ = this->create_wall_timer(100ms,
                                     std::bind(&TurtleBotController::controlLoop, this));
  }

};

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleBotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
  
}