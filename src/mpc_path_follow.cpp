/****************************************************************************
 * Adaptive Direct Shooting MPC Trajectory Following with Turtlebot3 using ROS2 and Google Ceres
 * Copyright (C) MIT 2022-2023 Quang Nhat Le
 *
 * @author Quang Nhat Le - quangle@umich.edu
 * @date   2023-Nov-14 (Last updated)
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <ceres/ceres.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>

// TODO: Polish the code and add its own library "mpc_obstacle_avoid.h" to reduce for lines for easy reading
#define PRINT_CMD(x) (std::cout << x << std::endl)
#define PRINT_CMD_2(x,y) (std::cout << x << y << std::endl)
#define CLAMP(x, lower, upper) ((x) < (lower) ? (lower) : ((x) > (upper) ? (upper) : (x)))

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// TODO: should change these parameters into struct config
// TODO: should make Get/Set functions to manually set goals
const int predictionHorizon = 10;
const double timeStep = 0.1;

const double lowerV = -1.0; // lower and upper constraints for control input linear
const double upperV = 1.0;
const double lowerW = -170.0 * M_PI / 180.0; // lower and upper constraints for control inputs angular
const double upperW = 170.0 * M_PI / 180.0 ;
const double targetV = 0.8;

const double goalWeight = 1.5; //0.3 seems ok
const double speedWeight = 0.9; //0.9 or 1.2

const double toleranceDist = 0.4;   //tolerance for robot to stop at goal
// const double safetyRadius = 0.6; // Safety radius to avoid obstacles (robot radius in this case - burger) 0.3 works ok
const double robot_stuck_flag_cons = 0.001;  // constant to prevent robot stucked

// TODO: path to save state and control history , configure so that everytimes run save in different name
const std::string fname="/home/quang_le/ros2_ws/src/hello/scripts/mpc_rover_state_path_follow_14_11_03.txt";  //just a directory, please change

/**
 * @brief Normalize angle between -pi and pi
 * 
 * @tparam T 
 * @param value 
 * @return T 
 */
template<typename T>
T norminalAngle(T value){
    while(*(double*) &value > M_PI || *(double*) &value < -M_PI){
        if(*(double*) &value > M_PI) value -= 2.0 * T(M_PI);
        else value += 2.0 *  T(M_PI);
    }
    return value;
}

/**
 * @brief Robot history trajectory
 * 
 */
struct Point
{
	float x;  //x coordinate
	float y;  // y-coordinate
    float v;  //linear velocity
    float w;
	float t;  // time
};

/**
 * @brief MPC struct Ceres cost functor - followed template in Ceres website
 * 
 */
struct MPCProblem {
  double x_init, y_init, yaw_init;
  double x_goal, y_goal;
  
  MPCProblem(double x_init, double y_init, double yaw_init, 
             double x_goal, double y_goal)
        // set the initial values for MPC
        // Note: const variables must be initialized here
      : x_init(x_init), y_init(y_init), yaw_init(yaw_init), 
        x_goal(x_goal), y_goal(y_goal)
       {}
  
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
        // TODO: should update to industry model with acceleration of both angular and linear
        // Update state using kinematic bicycle model
        // TODO: Why noy just use RK4 model instead of Forward Euler ? 
        //  Because w_k is held constant over the time step and the yaw dynamics represent pure integration, forward Euler is exact   
        x += v_k * cos(yaw) * dt;
        y += v_k * sin(yaw) * dt;
        yaw += norminalAngle( w_k * dt);
        
		// residual means differential by the target control input
	    //  goalWeight * (T(x_goal) - x) is differential for states (1/2)*x^{T}Qx
	    //  speedWeight * T( ceres::abs( T(targetV) - v_k)) is differential obtained from (1/2)*u^{T}Ru
	    // where Q = diag(speedWeight, speedWeight)

        // Compute residual (distance to goal)
        T x_diff = T(x_goal) - x;
        T y_diff = T(y_goal) - y;
        residual[0] = goalWeight * x_diff;
        residual[1] = goalWeight * y_diff;
        // residual[0] = T(0.0);
        // residual[1] = goalWeight * T(sqrt(x_diff * x_diff + y_diff * y_diff));

        //residual to obtain target linear velocity
        residual[2] = speedWeight * T( ceres::abs( T(targetV) - v_k));        
    }
    
    return true;
  }
};

/**
 * @brief Main MPC class of this code, inheritted from rclcpp
 * 
 */
class MPCController : public rclcpp::Node {
public:

  MPCController(void);
  
  //TODO: maybe try adding a destructor for this class ?
  ~MPCController(void){};

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);  
  void savePath(std::string fname, std::vector<Point> &path);
  void animationPlot(std::string fname);
  void process(void);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    double x_goal_, y_goal_;
    std::vector<Point> Trajectory; //to save rover trajectory
    rclcpp::Time node_start_time_;
    std::vector<double> prevOptimalU {0.0,0.0};

    geometry_msgs::msg::Twist velocity_msg; //current velocity

    // List of waypoints (desired positions)
    std::vector<std::pair<double, double>> waypoints_;
    size_t current_waypoint_;
    

};

/**
 * @brief Construct a new MPCController::MPCController object
 * 
 * @param x_goal 
 * @param y_goal 
 */
MPCController::MPCController(void):
    Node("mpc_path_follow"){

    // Create the velocity publisher
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscribe to odom topic for current robot state
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(& MPCController::odomCallback, this, std::placeholders::_1));

    // Initialize the node start time
    node_start_time_ = this->now();

    // Initialize waypoints
    // waypoints_ = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 3.0},{6.0, 3.0},{6.0,6.0}}; // Add more waypoints as needed
    waypoints_ = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 3.0},{6.0, 3.0},{6.0,6.0},{0.0, 0.0} }; // Loopback version
    // for (double x = -5.0; x <= 5.0; x += 0.5) {
    //     // Calculate y using a polynomial function (e.g., y = ax^2 + bx + c)
    //     double a = 0.05;
    //     double b = 0.06;
    //     double c = 0.07;
    //     double y = a * std::pow(x,3) + b * std::pow(x,2) + c * x ;

    //     // Add the (x, y) point as a waypoint
    //     waypoints_.push_back({x, y});
    // }

    current_waypoint_ = 0;

    // Set the initial desired position
    x_goal_ = waypoints_[current_waypoint_].first;
    y_goal_ = waypoints_[current_waypoint_].second;

    //TODO : we use get_clock to get the time, seems ok but we do have a /clock topic,
    // should try that as well

    //TODO : Also, if we want to switch between MANUAL control mode to AUTO mode,
    // maybe adding a joystick from manual controller here ?

    // process();
}

/**
 * @brief Call everytime subscribe to /odom
 * 
 * @param msg 
 */
void MPCController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){

    //get current time
    rclcpp::Time current_time = this->get_clock()->now();

    // Calculate the elapsed time in seconds
    float tt = (current_time - node_start_time_).seconds();

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
        new ceres::AutoDiffCostFunction<MPCProblem, 3, 1, 1>(
            new MPCProblem(x, y, theta, x_goal_, y_goal_));

    // Create a buffer for the control inputs
    // these are initialized using the previous control input
    // but more suitable initialization is required for fast optimization
    
    // double v = 0.0;  // Linear velocity
    // double w = 0.0;  // Angular velocity
    double v = prevOptimalU[0];  // Linear velocity
    double w = prevOptimalU[1];  // Angular velocity
    problem.AddResidualBlock(cost_function, nullptr, &v, &w);
    
    // set lower and upper constraints for the control inputs
    problem.SetParameterLowerBound(&v, 0, lowerV);
    problem.SetParameterUpperBound(&v, 0, upperV);
    problem.SetParameterLowerBound(&w, 0, lowerW);
    problem.SetParameterUpperBound(&w, 0, upperW);

    //TODO: Check to see whether Ceres supports doing inequality contraints with obs -Not really sure
    //Seems like we can write a for loop here for all obstacles with only 1 residual 

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false; //  if true, debug messages are shown
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << std::endl;

    if (std::abs(v) < robot_stuck_flag_cons ||
                     std::abs(msg->twist.twist.linear.x) < robot_stuck_flag_cons){
                    // to ensure the robot do not get stuck in
                    // best v=0 m/s (in front of an obstacle) and
                    //  best omega=0 rad/s (heading to the goal with
                    // angle difference of 0)
                    w = lowerW;
                    // v = lowerV;
    }

    // TODO: adding some clamp/calib to linear v ?
    //because we only need to control steering angle --- > no need because it is already in MPC ub and lb constraints

    // v = CLAMP(v,targetV,-targetV);

    // Publish control inputs to velocity topic
    // geometry_msgs::msg::Twist velocity_msg;
    velocity_msg.linear.x = v;
    velocity_msg.angular.z = w;
   
    // static bool isGoal = false;

    if (desired_dis <= toleranceDist)
    {
        velocity_msg.linear.x = 0.0;
        velocity_msg.angular.z = 0.0;
        velocity_pub_->publish(velocity_msg);
        RCLCPP_INFO(get_logger(), "FINISH SUCCESS with v = %f, z = %f \n", v, w);
        // v = 0.0;
        // w = 0.0;
        // isGoal = true;

        // Check if there are more waypoints
        if (current_waypoint_ < waypoints_.size()) {
            // Move to the next waypoint
            x_goal_ = waypoints_[current_waypoint_].first;
            y_goal_ = waypoints_[current_waypoint_].second;
            current_waypoint_++;
        } else {
            RCLCPP_INFO(this->get_logger(), "All waypoints reached!");
            // Stop the robot
            // x_goal_ = msg->pose.pose.position.x;
            // y_goal_ = msg->pose.pose.position.y;

            savePath(fname, Trajectory);
            animationPlot(fname);

            // Shutdown the ROS 2 node to terminate the program
            rclcpp::shutdown();
            return;

      }
    }
    else{
        // Print solution
        // RCLCPP_INFO(get_logger(), "MPC solution: v = %f, w = %f", v, w);
        RCLCPP_INFO(get_logger(), "Current input: v = %f, w = %f", velocity_msg.linear.x, velocity_msg.angular.z);
    }

    // if (!isGoal)
    // {
        velocity_pub_->publish(velocity_msg);
    // }

    //should use static_cast rather than C-type cast like this
    // saving the robot path using points
    Point  p;
    p.x= static_cast<float> (msg->pose.pose.position.x);
    p.y= static_cast<float> (msg->pose.pose.position.y);
    p.v = static_cast<float> (msg->twist.twist.linear.x) ;
    p.w = static_cast<float>( msg->twist.twist.angular.z) ;
    p.t = static_cast<float> (tt);
    Trajectory.push_back(p);

    //Update the prev optimal control input
    prevOptimalU[0] = v ; // Linear velocity
    prevOptimalU[1] = w; // Angular velocity
    // best_v = v;
    // best_w = w;
    

}


/**
 * @brief Save the trajectory and all states of robots to file to plot later
 * 
 * @param fname 
 * @param path 
 */
void MPCController::savePath(std::string fname, std::vector<Point> &path)
{
    std::ofstream MyFile(fname);

    for (size_t i = 0; i < path.size(); i++)
    {
        MyFile << path[i].x << " " << path[i].y << " " << path[i].v << " " << path[i].w << " " << path[i].t << std::endl;
    }

    MyFile.close();
}

/**
 * @brief Plotting using Gnuplot right after finishing navigation
 * 
 * @param fname 
 */
void MPCController::animationPlot(std::string fname){
    static FILE *gp;
    if(gp == NULL)
    {
        gp = popen("gnuplot -persist","w");
        fprintf(gp, "set colors classic\n");
        fprintf(gp, "set grid\n");
        fprintf(gp, "set xlabel Time [s]\n");
        fprintf(gp, "set tics font \"Arial, 14\"\n");
        fprintf(gp, "set xlabel font \"Arial, 14\"\n");
        fprintf(gp, "set ylabel font \"Arial, 14\"\n");
    }

    // Plot x versus y
    fprintf(gp, "set title 'Position (x vs. y)'\n");
    fprintf(gp, "plot \"%s\" using 1:2 with lines title 'x vs. y'\n", fname.c_str());
    fflush(gp);

    // Wait for user input (press Enter) before plotting the next graph
    printf("Press Enter to continue...\n");
    getchar();

    // Plot v versus t
    fprintf(gp, "set title 'Linear Velocity (v vs. t)'\n");
    fprintf(gp, "plot \"%s\" using 5:3 with lines title 'v vs. t'\n", fname.c_str());
    fflush(gp);

    // Wait for user input (press Enter) before plotting the next graph
    printf("Press Enter to continue...\n");
    getchar();

    //Plot w versus t
    fprintf(gp, "set title 'Angular Velocity (w vs. t)'\n");
    fprintf(gp, "plot \"%s\" using 5:4 with lines title 'w vs. t'\n", fname.c_str());
    fflush(gp);

    // Close gnuplot
    // fclose(gp);  // Do not close gnuplot here to keep the window open

    // Uncomment the line below if you want to automatically close gnuplot
    // int retVal = system("killall -9 gnuplot\n");
}



int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MPCController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
