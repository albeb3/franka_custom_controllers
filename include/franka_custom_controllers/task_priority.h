#ifndef TASK_PRIORITY_H
#define TASK_PRIORITY_H
#include <Eigen/Dense>
#include <std_msgs/String.h>
#include <array>
#include <string>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <math.h>

#include "franka_custom_controllers/my_utils.hpp"
#include "franka_custom_controllers/franka_data.h"



class task_priority{

    private:
    MissionManager& mission_manager_;
    tf::TransformListener& tf_listener_;
    iCAT_task_result icat_result_;
    franka_state& robot_state_;

    ros::NodeHandle nh_;
    //ros::Publisher cmd_vel_pub_;
    public:
  
    task_priority(ros::NodeHandle& nh, tf::TransformListener& tf_listener, MissionManager& mission_manager, franka_state& robot_state): nh_(nh), mission_manager_(mission_manager), tf_listener_(tf_listener), robot_state_(robot_state){
        //cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 10);
        //controller_return_mission_sub_ = nh_.subscribe<std_msgs::Bool>("/return_to_mission", 10, &robot_state::controllerReturnMissionCallback, this);
        //teleop_cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("/teleop/cmd_vel", 10, &robot_state::teleopCmdCallback, this);

    };
    ~task_priority()= default;
    
    void updateTaskPriority(){
        icat_result_.ydotbar = Eigen::MatrixXd::Zero(7,1);
        icat_result_.Q = Eigen::MatrixXd::Zero(7,7);
        icat_result_ = iCAT_Task( robot_state_.robot_state_.action_transitions.A_ee,
                                    robot_state_.robot_state_.jacobians.Jee,
                                    icat_result_.Q,
                                    robot_state_.robot_state_.task_references.xdot_g_pos,
                                    icat_result_.ydotbar,
                                    0.001, 0.01, 10);
    }

    void UpdateMissionPhase(){
        int phase = mission_manager_.GetPhase();
        mission_manager_.SetPhaseTime(0.02); // assuming update called at 50 Hz
        switch (phase)
        {
        case 0:
            {
            if(tf_listener_.frameExists("goal_frame")){
                mission_manager_.SetPhase(0,1);
                mission_manager_.SetPhaseTime(0.0); 
            }
            break;
            }
        case 1:
            {
            double distance_to_goal = robot_state_.getGoalDistance().head<3>().norm();
            std::cout << "Distance to goal: " << distance_to_goal << std::endl;
            if (distance_to_goal < 0.01 ){
                mission_manager_.SetPhase(1,2);
                mission_manager_.SetPhaseTime(0.0);
                robot_state_.goal_frame_exists_ = false; 
            }
            break;
            }
        case 2:
            {
            // Do nothing, mission accomplished
            break; 
            }
        }
    }

    Eigen::VectorXd getydotbarResult(){
        return icat_result_.ydotbar;
    }
  
  

};
#endif
