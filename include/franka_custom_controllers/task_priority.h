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
#include <hardware_interface/hardware_interface.h>
#include "franka_custom_controllers/my_utils.hpp"
#include "franka_custom_controllers/franka_data.h"
#include "franka_custom_controllers/mission_manager.hpp"



class task_priority{

    private:
    ros::NodeHandle node_handle_;
    std::shared_ptr<tf::TransformListener> tf_listener_;
    std::unique_ptr<MissionManager> mission_manager_;
    std::unique_ptr<franka_state> robot_state_;
    iCAT_task_result icat_result_;
    std::string arm_id_;
    hardware_interface::VelocityJointInterface* velocity_joint_interface_;
    std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
    
    void updateTaskPriority( ){
        icat_result_.ydotbar = Eigen::MatrixXd::Zero(7,1);
        icat_result_.Q = Eigen::MatrixXd::Identity(7,7);
        /*icat_result_.ydotbar = robot_state_->getJacobian("goal_move").transpose() * 
        (- robot_state_->getTaskReference("goal_move"));*/
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix("joint_limit"),
                                    robot_state_->getJacobian("joint_limit"),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference("joint_limit"),
                                    0.001, 0.01, 10);
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix("teleop"),
                                    robot_state_->getJacobian("teleop"),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference("teleop"),
                                    0.001, 0.01, 10);
        
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix("goal_move"),
                                    robot_state_->getJacobian("goal_move"),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference("goal_move"),
                                    0.001, 0.01, 10);
        icat_result_ = iCAT_Task( Eigen::MatrixXd::Identity(7,7),
                                    Eigen::MatrixXd::Identity(7,7),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    Eigen::MatrixXd::Zero(7,1),
                                    0.001, 0.01, 10);
        
        //std::cout << "ydotbar of "<< arm_id_<< " :" << icat_result_.ydotbar.transpose() << std::endl;

    }
    void sendCommands(){
        // compute control
        // allocate joint velocities
        Eigen::VectorXd q_dot(7);
        // compute joint velocities
        q_dot = getydotbarResult();
        // set joint velocities
        for (size_t i = 0; i < velocity_joint_handles_.size(); ++i) {
            velocity_joint_handles_[i].setCommand(q_dot(i));
        }
    }    
    void UpdateMissionPhase(){
        mission_manager_->SetPhaseTime(0.02);
        switch (mission_manager_->GetPhase())
        {
        case 0:
            
            {
                
            if(robot_state_->GetGoalFrameExists("goal_frame")){
                mission_manager_->SetPhase(mission_manager_->GetPhase(),1);
                 
            }
            else if(robot_state_->GetTeleopCmdReceived()){
                mission_manager_->SetPhase(mission_manager_->GetPhase(),3);
                robot_state_->SetReturnToMission(false);
                 
            }
            break;
            }
        case 1:
            {
            double distance_to_goal = robot_state_->getNormGoalDistance("goal_frame");
            std::cout << "Distance to goal: " << distance_to_goal << std::endl;
            if (distance_to_goal < 0.001 ){
                mission_manager_->SetPhase(mission_manager_->GetPhase(),2);
               
            }
            else if (robot_state_->GetTeleopCmdReceived()){
                mission_manager_->SetPhase(mission_manager_->GetPhase(),3);
                robot_state_->SetReturnToMission(false);
                
            }
            
            break;
            }
        case 2:
            {   
                robot_state_->SetTaskReferncesToZero();
                double distance_to_goal = robot_state_->getNormGoalDistance("goal_frame");
                std::cout << "Distance to goal in stop phase: " << distance_to_goal << std::endl;
                if (distance_to_goal >= 0.001 && robot_state_->GetReturnToMission()){
                    mission_manager_->SetPhase(mission_manager_->GetPhase(),1);
                }
                else if (robot_state_->GetTeleopCmdReceived()){
                    mission_manager_->SetPhase(mission_manager_->GetPhase(),3);
                    robot_state_->SetReturnToMission(false);
                    
                }
                
            //std::cout << "In stop phase" << std::endl;
            // Do nothing, mission accomplished
            break; 
            }
        case 3:
            {
            double distance_to_teleop_goal = robot_state_->getNormGoalDistance("teleop");
            std::cout << "Distance to teleop goal: " << distance_to_teleop_goal << std::endl;
            if (robot_state_->GetReturnToMission())
                {
                std::cout << "Returning to mission from teleop" << std::endl;
                std::cout << "Previous phase: " << mission_manager_->GetPreviousPhase() << std::endl;
                mission_manager_->SetPhase(mission_manager_->GetPhase(),mission_manager_->GetPreviousPhase());
                robot_state_->SetTeleopCmdReceived(false);
                
                }
            else if (distance_to_teleop_goal < 0.001 ){
                mission_manager_->SetPhase(mission_manager_->GetPhase(),2);
                robot_state_->SetTeleopCmdReceived(false);
                
            }
            break;
            }
        }
    }


    Eigen::VectorXd getydotbarResult(){
        return icat_result_.ydotbar;
    }
    
    public:
    task_priority(ros::NodeHandle& nh): node_handle_(nh){};
    ~task_priority()= default;
    
    bool  init( hardware_interface::RobotHW* robot_hardware){

        velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
        if (velocity_joint_interface_ == nullptr) {
            ROS_ERROR(
                "JointVelocityController: Error getting velocity joint interface from hardware!");
            return false;
        }
        std::string arm_id;
        if (!node_handle_.getParam("arm_id", arm_id)) {
            ROS_ERROR("JointVelocityController: Could not get parameter arm_id");
            return false;
        }
        arm_id_ = arm_id;
        std::vector<std::string> joint_names;
        if (!node_handle_.getParam("joint_names", joint_names)) {
            ROS_ERROR("JointVelocityController: Could not parse joint names");
        }
        if (joint_names.size() != 7) {
            ROS_ERROR_STREAM("JointVelocityController: Wrong number of joint names, got "
                            << joint_names.size() << " instead of 7 names!");
            return false;
        }
        velocity_joint_handles_.resize(7);
        for (size_t i = 0; i < 7; ++i) {
            try {
            velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "JointVelocityController: Exception getting joint handles: " << ex.what());
            return false;
            }
        }

        mission_manager_ = std::make_unique<MissionManager>();
        tf_listener_ = std::make_shared<tf::TransformListener>();
        robot_state_ = std::make_unique<franka_state>(node_handle_, *mission_manager_, *tf_listener_);
        
        return robot_state_->init(robot_hardware, arm_id);
    };
    void starting(){
        // initialize mission manager
        mission_manager_->Starting();
        robot_state_->starting();
    }
    void update(){
        robot_state_->update();
        updateTaskPriority( );
        robot_state_->updateState();
        sendCommands();
        UpdateMissionPhase();

    };
    
  
  

};
#endif
