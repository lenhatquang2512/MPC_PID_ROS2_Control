#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
// #include "std_msgs/msg/float32_multi_array.hpp"
#include <ceres/ceres.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

//should change these parameters into struct config
const int predictionHorizon = 10;
const double timeStep = 0.1;

//goal position should be set by the user
const double targetX = 5.0; 
const double targetY = -3.0;

const double lowerV = -0.5; // lower and upper constraints for control input linear
const double upperV = 1.0;
const double lowerW = -70.0 * M_PI / 180.0; // lower and upper constraints for control inputs angular
const double upperW = 70.0 * M_PI / 180.0 ;
const double targetV = 0.5;

const double goalWeight = 0.9;
const double obsWeight = 1.0;
const double speedWeight = 0.7;

const double lookaheadDist_ = 0.5;
const double safetyRadius = 0.1; // Safety radius to avoid obstacles (robot radiua in this case - burger)
const double robot_stuck_flag_cons = 0.001;  // constant to prevent robot stucked

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
  const std::vector<std::vector<double>> obs_list;
// const std::vector<double>& scan_ranges;
  
  MPCProblem(double x_init, double y_init, double yaw_init, 
             double x_goal, double y_goal, const std::vector<std::vector<double>> obs_list)
      : x_init(x_init), y_init(y_init), yaw_init(yaw_init), 
        x_goal(x_goal), y_goal(y_goal) ,
        obs_list(obs_list) {}
  
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
        // should update to industru model with acceleration of both angular and linear
        // Update state using kinematic model
        x += v_k * cos(yaw) * dt;
        y += v_k * sin(yaw) * dt;
        yaw += norminalAngle( w_k * dt);

        // Compute residual (distance to goal)
        T x_diff = T(x_goal) - x;
        T y_diff = T(y_goal) - y;
        residual[0] = goalWeight * x_diff;
        residual[1] = goalWeight * y_diff;
        // residual[0] = T(0.0);
        // residual[1] = goalWeight * T(sqrt(x_diff * x_diff + y_diff * y_diff));

        //obstacle avoidance
        // T obstacle_residual = T(0.0);
        // for (std::size_t i = 0; i < obs_list.size(); ++i) {
        //     T dist = T (sqrt((x - obs_list[i][0])*(x - obs_list[i][0]) + (y - obs_list[i][1])*(y - obs_list[i][1]))) ;
        //     obstacle_residual += ceres::abs(T(safetyRadius) - dist);
        // }

        T cost = T(0.0);
        T min_dist = T(1e6);
        for(const auto& obs : obs_list){
            T dist = sqrt((x - T (obs[0]) )*(x  - T( obs[0]) ) + (y - T (obs[1]) )*(y - T (obs[1])));
            min_dist = std::min(min_dist, dist);
        }
        if (min_dist <= safetyRadius)  // not too closed
        {
            cost = T (INFINITY);
        }
        else{
            cost = T (1.0 / min_dist);
        }
        residual[2] = obsWeight * cost;
        
        //residual to obtain target linear velocity
        residual[3] = speedWeight * T( ceres::abs( T(targetV) - v_k));

    }
    
    return true;
  }
};

class MPCController : public rclcpp::Node {
public:

  MPCController(double x_goal, double y_goal);

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);  
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg); 
  // void process(void);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    double x_goal_, y_goal_;
    std::vector<std::vector<double>> obs_list;
    // sensor_msgs::msg::LaserScan scan;
    // geometry_msgs::msg::Twist current_velocity;

};

MPCController::MPCController(double x_goal, double y_goal):
    Node("mpc_obstacle_avoid"), x_goal_(x_goal), y_goal_(y_goal){

    // Create the velocity publisher
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscribe to odom topic for current robot state
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(& MPCController::odomCallback, this, std::placeholders::_1));

    // Subscribe to laser scan topic for obstacle detection
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&MPCController::laserCallback, this, std::placeholders::_1));

    //should add IMU

    // process();
}

void MPCController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){

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
        new ceres::AutoDiffCostFunction<MPCProblem, 4, 1, 1>(
            new MPCProblem(x, y, theta, x_goal_, y_goal_, obs_list));
    double v = 0.0;  // Linear velocity
    double w = 0.0;  // Angular velocity
    problem.AddResidualBlock(cost_function, nullptr, &v, &w);
    
    // set lower and upper constraints for the control inputs
    problem.SetParameterLowerBound(&v, 0, lowerV);
    problem.SetParameterUpperBound(&v, 0, upperV);
    problem.SetParameterLowerBound(&w, 0, lowerW);
    problem.SetParameterUpperBound(&w, 0, upperW);

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false; //  if true, debug messages are shown
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << std::endl;

    if (std::abs(v) < robot_stuck_flag_cons ||
                     std::abs(msg->twist.twist.linear.x) < robot_stuck_flag_cons)
                    // to ensure the robot do not get stuck in
                    // best v=0 m/s (in front of an obstacle) and
                    //  best omega=0 rad/s (heading to the goal with
                    // angle difference of 0)
                    w = lowerW;

    // Publish control inputs to velocity topic
    geometry_msgs::msg::Twist velocity_msg;
    velocity_msg.linear.x = v;
    velocity_msg.angular.z = w;

    static bool isGoal = false;

    if (desired_dis <= lookaheadDist_)
    {
        velocity_msg.linear.x = 0.0;
        velocity_msg.angular.z = 0.0;
        velocity_pub_->publish(velocity_msg);
        RCLCPP_INFO(get_logger(), "FINISH SUCCESS with v = %f, z = %f \n", v, w);
        v = 0.0;
        w = 0.0;
        isGoal = true;
        // return;
    }
    else{
        // Print solution
        RCLCPP_INFO(get_logger(), "MPC solution: v = %f, w = %f", v, w);
    }

    if (!isGoal)
    {
        velocity_pub_->publish(velocity_msg);
    }
    

}

void MPCController::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    //list of obstacles
    // std::vector<std::vector<double>> obs_list;
    obs_list.clear();

    for (int i = 0; i < msg->ranges.size(); i++)
    {
        // Check if the range is valid
        if (!std::isnan(msg->ranges[i]))
        {
            double angle = (double) msg->angle_min + i * msg->angle_increment;

            // Extract x and y coordinates from the scan data
            double x = (double) msg->ranges[i] * cos(angle);
            double y = (double) msg->ranges[i] * sin(angle);
            std::vector<double> obs_state = {x, y};
            obs_list.push_back(obs_state);
        }
    }

}

// void MPCController::process(void){
     // //to do something else, maybe connect with the custom Plugin Nav2 control ?
// }

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MPCController>(targetX, targetY);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}