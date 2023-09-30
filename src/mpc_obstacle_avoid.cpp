/****************************************************************************
 * Adaptive MPC DWA-based Obstacles Avoidance with Turtlebot3 using ROS2 and Google Ceres
 * Copyright (C) MIT 2022-2023 Quang Nhat Le
 *
 * @author Quang Nhat Le - quangle@umich.edu
 * @date   2023-Sep-23 (Last updated)
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
// #include "std_msgs/msg/float32_multi_array.hpp"
#include <ceres/ceres.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <fstream>
// TODO: Polish the code and add its own library "mpc_obstacle_avoid.h" to reduce for lines for easy reading

#define PRINT_CMD(x) (std::cout << x << std::endl)
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

//goal position should be set by the user (ex: 5,-3)
const double targetX = 6.0; 
const double targetY = 6.0;

const double lowerV = -0.5; // lower and upper constraints for control input linear
const double upperV = 1.0;
const double lowerW = -40.0 * M_PI / 180.0; // lower and upper constraints for control inputs angular
const double upperW = 40.0 * M_PI / 180.0 ;
// const double lowerW = -1.0;
// const double upperW = 1.0;
const double targetV = 0.4;

const double goalWeight = 0.3;
const double obsWeight = 1.1;
const double speedWeight = 0.9;

const double lookaheadDist_ = 0.4;
const double safetyRadius = 0.3; // Safety radius to avoid obstacles (robot radiua in this case - burger)
const double robot_stuck_flag_cons = 0.001;  // constant to prevent robot stucked

//handle laser scan too close or too far from obstacles in Gazebo
const double min_threshold = 0.35;
// const double thres_touch_obs = 1.2;
const double close_r_tuned = 0.001;
const double far_r_tuned = 100.0;

// TODO: path to save state and control history , configure so that everytimes run save in different name
const std::string fname="/home/quang_le/ros2_ws/src/hello/scripts/mpc_rover_state_obs_avoid_3.txt";  //just a directory, please change

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
  const std::vector<std::vector<double>> obs_list;
// const std::vector<double>& scan_ranges;
  
  MPCProblem(double x_init, double y_init, double yaw_init, 
             double x_goal, double y_goal, const std::vector<std::vector<double>> obs_list)
        // set the initial values for MPC
        // Note: const variables must be initialized here
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
        // should update to industry model with acceleration of both angular and linear
        // Update state using kinematic model
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
            // T dist = sqrt((T (obs[0]) )*(T( obs[0]) ) + ( T (obs[1]) )*( T (obs[1])));
            min_dist = std::min(min_dist, dist);
        }
        if (min_dist <= safetyRadius)  // not too closed
        {
            // cost = T (INFINITY);
            cost = T(1e6);
        }
        else{
            cost = T (1.0 / min_dist);
        }
        residual[2] = obsWeight * cost;

        //residual to obtain target linear velocity
        residual[3] = speedWeight * T( ceres::abs( T(targetV) - v_k));

        //FIXED STABLE OBSTACLES CASE (TEST WITH 1)
        // T dist1 = T (sqrt((x - T (2.5) )*(x  - T(2.5 ) ) + (y - T (-1.5) )*(y - T (-1.5))));
        // residual[2] = T (1.0 /dist1);

        // T dist2 = T (sqrt((x - T (3.5) )*(x  - T(3.5 ) ) + (y - T (-0.5) )*(y - T (-0.5))));
        // residual[3] = T (1.0 /dist2);

        // T dist3 = T (sqrt((x - T (3.0) )*(x  - T(3.0 ) ) + (y - T (-1.0) )*(y - T (-1.0))));
        // residual[4] = T (1.0 /dist3);
        


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

  MPCController(double x_goal, double y_goal);
  
  //TODO: maybe try adding a destructor for this class ?

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);  
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg); 
  void printObs(const std::vector<std::vector<double>>& vec);
  void savePath(std::string fname, std::vector<Point> &path);
  void animationPlot(std::string fname);

  // void process(void);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    double x_goal_, y_goal_;
    std::vector<std::vector<double>> obs_list;
    std::vector<Point> Trajectory; //to save rover trajectory
    rclcpp::Time node_start_time_;

    // sensor_msgs::msg::LaserScan scan;
    // geometry_msgs::msg::Twist current_velocity;

};

/**
 * @brief Construct a new MPCController::MPCController object
 * 
 * @param x_goal 
 * @param y_goal 
 */
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

    // Initialize the node start time
    node_start_time_ = this->now();

    // TODO: should add IMU

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
        new ceres::AutoDiffCostFunction<MPCProblem, 4, 1, 1>(
            new MPCProblem(x, y, theta, x_goal_, y_goal_, obs_list));

    // TODO: create a buffer for the control inputs
    // these are initialized using the previous control input
    // but more suitable initialization is required for fast optimization
    
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
                     std::abs(msg->twist.twist.linear.x) < robot_stuck_flag_cons){
                    // to ensure the robot do not get stuck in
                    // best v=0 m/s (in front of an obstacle) and
                    //  best omega=0 rad/s (heading to the goal with
                    // angle difference of 0)
                    w = lowerW;
                    // v = lowerV;
    }

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

        savePath(fname, Trajectory);
        animationPlot(fname);
        // Shutdown the ROS 2 node to terminate the program
        rclcpp::shutdown();
        return;
    }
    else{
        // Print solution
        RCLCPP_INFO(get_logger(), "MPC solution: v = %f, w = %f", v, w);
    }

    if (!isGoal)
    {
        velocity_pub_->publish(velocity_msg);
    }

    // for(const auto& obs : obs_list){
    //     double dist = sqrt((double(obs[0]) )*(double( obs[0]) ) + (double (obs[1]) )*( double(obs[1])));
    //     std::cout << "Dist = " << dist <<  " x_obs = " << obs[0] << " y_obs = " << obs[1] << std::endl;
    // }
    

    //TODO: should use static_cast rather than C-type cast like this
    // saving the robot path using points
    Point  p;
    p.x= (float) msg->pose.pose.position.x;
    p.y= (float) msg->pose.pose.position.y;
    p.v = (float) msg->twist.twist.linear.x ;
    p.w = (float) msg->twist.twist.angular.z ;
    p.t = (float) tt;

    Trajectory.push_back(p);
    

}

/**
 * @brief Call everytime subscribe to /scan
 * 
 * @param msg 
 */
void MPCController::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    //list of obstacles
    // std::vector<std::vector<double>> obs_list;
    obs_list.clear();

    // double min_range = msg->range_min;  //ah for gazebo this is just 0.12m
    double max_range = msg->range_max;   //and this is 3.5m, fixed

    // double min_threshold = 0.4;
    // double thres_touch_obs = 1.2;
    double r;
    // double r_prev = 0.0;

    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
        double range = msg->ranges[i];
        // std::cout << "Min : " << min_range << " r = " << range << " Max : " << max_range << std::endl;

        if (min_threshold <= range && range <= max_range) {
            r = range;

        }else if (std::isfinite(range) && range < min_threshold){
            // r = 0.001;
            r = close_r_tuned;

        }else if (!std::isfinite(range) && range < 0) {
            // Object too close to measure.
            // Set x and y to 0
            // double x = 0.0;
            // double y = 0.0;
            // std::vector<double> obs_state = {x, y};
            // obs_list.push_back(obs_state);
            r = 0.0;
        } else if (!std::isfinite(range) && range > 0) {
            // No objects detected in range.
            // continue;
            r = (double) far_r_tuned;
        } else if (std::isnan(range)) {
            // This is an erroneous, invalid, or missing measurement.
            // Handle or ignore.
            continue;
        } else {
            // The sensor reported these measurements as valid, but they are discarded
            // per the limits defined by minimum_range and maximum_range.
            // Handle or ignore.
            continue;
        }
        
        //// very strong condition to force the robot not touch obstacle
        ////but trade off is it is very unstable
        // if((i >0) && (r_prev == (double)0.0) && ((r-r_prev) < thres_touch_obs)){
        //     r = 0.001;
        // }

        // This is a valid measurement.
        double angle = (double) msg->angle_min + i * msg->angle_increment;

        // Extract x and y coordinates from the scan data
        double x = (double) r * cos(angle);
        double y = (double) r * sin(angle);
        std::vector<double> obs_state = {x, y};
        obs_list.push_back(obs_state);

        // r_prev = r;

        // TODO: maybe adding Kalman filter here ?

        // // Check if the range is valid
        // if (!std::isnan(msg->ranges[i]))
        // {
        //     double angle = (double) msg->angle_min + i * msg->angle_increment;

        //     // Extract x and y coordinates from the scan data
        //     double x = (double) msg->ranges[i] * cos(angle);
        //     double y = (double) msg->ranges[i] * sin(angle);
        //     std::vector<double> obs_state = {x, y};
        //     obs_list.push_back(obs_state);
        // }
    }

    // printObs(obs_list);

}

/**
 * @brief Just for printing obstacles coordinates
 * 
 * @param vec 
 */
void MPCController::printObs(const std::vector<std::vector<double>>& vec)
{
    for (const auto& inner_vec : vec)
    {
        for (const auto& value : inner_vec)
        {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
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
    fprintf(gp, "set title 'Angular Velocity (v vs. t)'\n");
    fprintf(gp, "plot \"%s\" using 5:4 with lines title 'v vs. t'\n", fname.c_str());
    fflush(gp);

    // Close gnuplot
    // fclose(gp);  // Do not close gnuplot here to keep the window open

    // Uncomment the line below if you want to automatically close gnuplot
    // int retVal = system("killall -9 gnuplot\n");
}

// void MPCController::process(void){
     // //TODO: something else, maybe connect with the custom Plugin Nav2 control ?
// }

/**
 * @brief Calling the class node object or whatever it is, not sure make_shared means
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MPCController>(targetX, targetY);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
