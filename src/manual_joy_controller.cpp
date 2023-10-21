/****************************************************************************
 * Manual Joystick Control with Turtlebot3 using ROS2 
 * Copyright (C) MIT 2022-2023 Quang Nhat Le
 *
 * @author Quang Nhat Le - quangle@umich.edu
 * @date   2023-Sep-23 (Last updated)
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <ceres/ceres.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <fstream>
#include <sensor_msgs/msg/joy.hpp>
// #include "std_msgs/msg/float32_multi_array.hpp"

#define PRINT_CMD(x) (std::cout << x << std::endl)
#define CLAMP(x, lower, upper) ((x) < (lower) ? (lower) : ((x) > (upper) ? (upper) : (x)))

// TODO: path to save state and control history , configure so that everytimes run save in different name
// const std::string fname="/home/pgurram/ros2_ws/src/MPC_PID_ROS2_Control/scripts/mpc_rover_state_obs_avoid_3.txt";  //just a directory, please change

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

class JoyController : public rclcpp::Node {
public:

  JoyController(double x_goal, double y_goal);
  
  //TODO: maybe try adding a destructor for this class ?

//   void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);  
  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);  
  void savePath(std::string fname, std::vector<Point> &path);
  void animationPlot(std::string fname);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    double x_goal_, y_goal_;
    std::vector<std::vector<double>> obs_list;
    std::vector<Point> Trajectory; //to save rover trajectory
    rclcpp::Time node_start_time_;

    // sensor_msgs::msg::LaserScan scan;
    // geometry_msgs::msg::Twist current_velocity;
};

/**
 * @brief Construct a new JoyController::JoyController object
 * 
 * @param x_goal 
 * @param y_goal 
 */
JoyController::JoyController(double x_goal, double y_goal):
    Node("joy_controller_node"), x_goal_(x_goal), y_goal_(y_goal){

    // Create the velocity publisher
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscribe to odom topic for current robot state
    // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "odom", 10,
    //     std::bind(& JoyController::odomCallback, this, std::placeholders::_1));

    // Subscribe to laser scan topic for obstacle detection
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoyController::JoyCallback, this, std::placeholders::_1));

    // Initialize the node start time
    node_start_time_ = this->now();

    // process();
}

void JoyController::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg){
    int buttons_5;
    int buttons_4;
    int buttons_1;

    buttons_4 = msg->buttons[4];
    buttons_5 = msg->buttons[5];
    buttons_1 = msg->buttons[1];
    // buttons_2 = msg->buttons[4];
    // Publish control inputs to velocity topic
    geometry_msgs::msg::Twist velocity_msg;

    std::cout << "This is the result of button5: " << buttons_5 
    << "But 4: " << buttons_4 << " But 1 : " << buttons_1 << std::endl;
    std::cout << "Linear v = " <<  velocity_msg.linear.x << " Angular z = " <<  velocity_msg.angular.z << std::endl;


    if (buttons_4 == true && buttons_5 == false) // move linear
    {
            velocity_msg.linear.x = 0.5;
            velocity_msg.angular.z = 0.0;
            velocity_pub_->publish(velocity_msg);
        
    }else if(buttons_4 == false && buttons_5 == true){  //rotate

            velocity_msg.linear.x = 0.0;
            velocity_msg.angular.z = 0.2;
            velocity_pub_->publish(velocity_msg);
    }else if(buttons_4 == false && buttons_5 == false){  //stops
           if(buttons_1 == true){
            velocity_msg.linear.x = 0.0;
            velocity_msg.angular.z = 0.0;
            velocity_pub_->publish(velocity_msg);
           }
    }
}

/**
 * @brief Save the trajectory and all states of robots to file to plot later
 * 
 * @param fname 
 * @param path 
 */
void JoyController::savePath(std::string fname, std::vector<Point> &path)
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
void JoyController::animationPlot(std::string fname){
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

/**
 * @brief Calling the class node object or whatever it is, not sure make_shared means
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyController>(0, 0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
