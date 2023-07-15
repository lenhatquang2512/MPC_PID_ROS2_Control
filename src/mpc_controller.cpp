#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <ceres/ceres.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

const int predictionHorizon = 10;
const double timeStep = 0.1;
// const int dimX = 3;
// const int dimU = 2;
// const int uNums = predictionHorizon * dimU; // numbers of predicted control inputs
const double targetX = 5.0; 
const double targetY = -2.0;
const double lowerU = -1.0; // lower and upper constraints for control inputs
const double upperU = 1.0;
const double regularizerWeight = 0.3;
const double lookaheadDist_ = 0.05;

template<typename T>
T norminalAngle(T value){
    while(*(double*) &value > M_PI || *(double*) &value < -M_PI){
        if(*(double*) &value > M_PI) value -= 2.0 * T(M_PI);
        else value += 2.0 *  T(M_PI);
    }
    return value;
}

struct MPCProblem {
  double x_init, y_init, yaw_init;
  double x_goal, y_goal;
  
  MPCProblem(double x_init, double y_init, double yaw_init, double x_goal, double y_goal)
      : x_init(x_init), y_init(y_init), yaw_init(yaw_init), x_goal(x_goal), y_goal(y_goal) {}
  
  template<typename T>
  bool operator()(const T* const v, const T* const w, T* residual) const {
    // State variables
    T x = T(x_init);
    T y = T(y_init);
    T yaw = T(yaw_init);
    
    // Control inputs
    T dt = T(timeStep); // Time step
    T v_k = T(v[0]);
    T w_k = T(w[0]);
    
    // MPC iterations
    for (int i = 0; i < predictionHorizon; ++i) {
      // Update state using kinematic model
      x += v_k * cos(yaw) * dt;
      y += v_k * sin(yaw) * dt;
      yaw += norminalAngle( w_k * dt);

    // Compute residual (distance to goal)
    T x_diff = T(x_goal) - x;
    T y_diff = T(y_goal) - y;
    residual[0] = regularizerWeight * x_diff;
    residual[1] = regularizerWeight * y_diff;
    }
    
    return true;
  }
};

class MPCController : public rclcpp::Node {
public:
  MPCController(double x_goal, double y_goal)
     : Node("mpc_controller"), x_goal_(x_goal), y_goal_(y_goal) {
    
    // Create the velocity publisher
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscribe to odom topic for current robot state
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(& MPCController::odomCallback, this, std::placeholders::_1));
  }

private:

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    const geometry_msgs::msg::Quaternion& quat = msg->pose.pose.orientation;
    // Convert the quaternion to Euler angles (roll, pitch, yaw)
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    double theta = yaw; // Extract yaw from orientation quaternion

    // Calculate the errors
    double error_x_ =  x - x_goal_ ;
    double error_y_ =  y - y_goal_ ;

    double desired_dis = sqrt(pow(error_x_,2) + pow(error_y_,2));

    // Solve MPC problem
    //         // <MPCCostFunctor, uNums, uNums>
    //         // MPCCostFunctor: cost function
    //         // uNums (first): numbers of residual
    //         // uNums (second): numbers of u (see operator written in above)
    //         // if your operator has several variables,
    //         // e.g., operator()(const T *const x, const T *const u, T *residual)
    //         // you should call <MPCCostFunctor, num of residual, num of x, num of u>
    //         return (new ceres::AutoDiffCostFunction<MPCCostFunctor, uNums, uNums>
    //                (new MPCCostFunctor()));
    ceres::Problem problem;
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<MPCProblem, 2, 1, 1>(
            new MPCProblem(x, y, theta, x_goal_, y_goal_));
    double v = 0.0;  // Linear velocity
    double w = 0.0;  // Angular velocity
    problem.AddResidualBlock(cost_function, nullptr, &v, &w);
    
    // set lower and upper constraints for the control inputs
    problem.SetParameterLowerBound(&v, 0, lowerU );
    problem.SetParameterUpperBound(&v, 0, upperU);
    problem.SetParameterLowerBound(&w, 0, lowerU);
    problem.SetParameterUpperBound(&w, 0, upperU);

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false; //  if true, debug messages are shown
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << std::endl;

    // Publish control inputs to velocity topic
    geometry_msgs::msg::Twist velocity_msg;
    velocity_msg.linear.x = v;
    velocity_msg.angular.z = w;

    if (desired_dis <= lookaheadDist_)
    {
        velocity_msg.linear.x = 0.0;
        velocity_msg.angular.z = 0.0;
        velocity_pub_->publish(velocity_msg);
        RCLCPP_INFO(get_logger(), "FINISH SUCCESS with v = %f, z = %f \n", v, w);
    }
    else{
        // Print solution
        RCLCPP_INFO(get_logger(), "MPC solution: v = %f, w = %f", v, w);
    }

    velocity_pub_->publish(velocity_msg);
  }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double x_goal_, y_goal_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Provide the goal coordinates
  //   double x_goal = 3.0;
  //   double y_goal = -1.0;

  auto node = std::make_shared<MPCController>(targetX, targetY);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}