#pragma once

#include "ceres/ceres.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

// namespace rostek_followline{

    struct MPCConfig{
        double w_s_cte = 2.;
        double w_s_etheta = 5.;
        double w_s_vel = 5.;
        double w_f_d = 11.;
        double w_f_angular = 12.;
        double w_f_speed = 10.;
        double d_t = 0.1;
        double speed = 0.8;
    };

    struct RobotConfig{
        double max_vel = 1.0;
        double max_angular = 1.0;
    };

    class MPCCostFunctor{
    private:
        // MPCCostFunctor* pre_step_ = nullptr; // save pointer of pre step
        bool stop_ = false; //flag for goal, if true, robot must stop in goal
        // geometry_msgs::msg::Point goal_; // goal of this node
        // geometry_msgs::msg::Twist* start_vel_;// save start velocity of mpc

        double x_init , y_init,  theta_init;

        MPCConfig* mpc_config_; // save config mpc
        RobotConfig* robot_config_; // save robot feature

    public:
        MPCCostFunctor(double x_init, double y_init, double theta_init);
        ~MPCCostFunctor();

        template<typename T>
        bool operator()(const T* const v, const T* const w, T* residual) const;

        // void setGoal(double x, double y);
    
        // void setPreNode(MPCCostFunctor* pre_step);

        // void clearGoal();

        // void setConfig(geometry_msgs::msg::Twist* start_vel, MPCConfig* mpc_config, RobotConfig* robot_config);

        // std::shared_ptr<geometry_msgs::msg::Pose2D> robot_pose_; //save pose after k*dt
        // std::shared_ptr<geometry_msgs::msg::Twist> robot_vel_; //save vel after k*dt

    };
// }

