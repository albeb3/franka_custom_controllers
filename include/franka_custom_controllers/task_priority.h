#ifndef TASK_PRIORITY_H
#define TASK_PRIORITY_H
#include <Eigen/Dense>
#include <std_msgs/String.h>
#include <array>
#include <string>
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
    std::unique_ptr<MissionManager> mission_manager_;
    std::unique_ptr<franka_state> robot_state_;
    iCAT_task_result icat_result_;
    std::string arm_id_;
    hardware_interface::VelocityJointInterface* velocity_joint_interface_;
    std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
    
    void updateTaskPriority( ){
        icat_result_.ydotbar = Eigen::MatrixXd::Zero(7,1);
        icat_result_.Q = Eigen::MatrixXd::Identity(7,7);
      /* 
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix(Task_name::JOINT_4_CUBE),
                                    robot_state_->getJacobian(Task_name::JOINT_4_CUBE),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference(Task_name::JOINT_4_CUBE),
                                    0.001, 0.01, 10); 
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix(Task_name::JOINT_3_CUBE),
                                    robot_state_->getJacobian(Task_name::JOINT_3_CUBE),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference(Task_name::JOINT_3_CUBE),
                                    0.001, 0.01, 10);
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix(Task_name::JOINT_2_CUBE),
                                    robot_state_->getJacobian(Task_name::JOINT_2_CUBE),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference(Task_name::JOINT_2_CUBE),
                                    0.001, 0.01, 10);
                                    
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix(Task_name::HAND_OTHM),
                                    robot_state_->getJacobian(Task_name::HAND_OTHM),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference(Task_name::HAND_OTHM),
                                    0.001, 0.01, 10);
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix(Task_name::FINGERLEFT_OTHM),
                                    robot_state_->getJacobian(Task_name::FINGERLEFT_OTHM),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference(Task_name::FINGERLEFT_OTHM),
                                    0.001, 0.01, 10);
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix(Task_name::FINGERRIGHT_OTHM),
                                    robot_state_->getJacobian(Task_name::FINGERRIGHT_OTHM),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference(Task_name::FINGERRIGHT_OTHM),
                                    0.001, 0.01, 10);
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix(Task_name::JOINT_7_OTHM),
                                    robot_state_->getJacobian(Task_name::JOINT_7_OTHM),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference(Task_name::JOINT_7_OTHM),
                                    0.001, 0.01, 10);
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix(Task_name::JOINT_6_OTHM),
                                    robot_state_->getJacobian(Task_name::JOINT_6_OTHM),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference(Task_name::JOINT_6_OTHM),
                                    0.001, 0.01, 10);
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix(Task_name::JOINT_5_OTHM),
                                    robot_state_->getJacobian(Task_name::JOINT_5_OTHM),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference(Task_name::JOINT_5_OTHM),
                                    0.001, 0.01, 10);
       
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix(Task_name::JOINT_4_OTHM),
                                    robot_state_->getJacobian(Task_name::JOINT_4_OTHM),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference(Task_name::JOINT_4_OTHM),
                                    0.001, 0.01, 10);
        
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix(Task_name::JOINT_LIMIT),
                                    robot_state_->getJacobian(Task_name::JOINT_LIMIT),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference(Task_name::JOINT_LIMIT),
                                    0.001, 0.01, 10);
               
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix(Task_name::TELEOP),
                                    robot_state_->getJacobian(Task_name::TELEOP),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference(Task_name::TELEOP),
                                    0.001, 0.01, 10);
        */
        icat_result_ = iCAT_Task( robot_state_->getActivationMatrix(Task_name::GOAL_MOVE),
                                    robot_state_->getJacobian(Task_name::GOAL_MOVE),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    robot_state_->getTaskReference(Task_name::GOAL_MOVE),
                                    0.001, 0.01, 10);
        
        icat_result_ = iCAT_Task( Eigen::MatrixXd::Identity(7,7),
                                    Eigen::MatrixXd::Identity(7,7),
                                    icat_result_.Q,
                                    icat_result_.ydotbar,
                                    Eigen::MatrixXd::Zero(7,1),
                                    0.001, 0.01, 10);
        
        //std::cout << "ydotbar of "<< arm_id_<< " :" << icat_result_.ydotbar.transpose() << std::endl;

    }
   
    void updateMissionPhase(){
        mission_manager_->setPhaseTime(0.02);
        switch (mission_manager_->getPhase())
        {
        case 0:
            
            {
                
            if(robot_state_->getGoalFrameExists("goal_frame")){
                mission_manager_->setPhase(mission_manager_->getPhase(),1);
                 
            }
            else if(robot_state_->getTeleopCmdReceived()){
                mission_manager_->setPhase(mission_manager_->getPhase(),3);
                robot_state_->setReturnToMission(false);
                 
            }
            break;
            }
        case 1:
            {
            double distance_to_goal = robot_state_->getNormGoalDistance("goal_frame");
            //std::cout << "Distance to goal: " << distance_to_goal << std::endl;
            if (distance_to_goal < 0.0001 ){
                mission_manager_->setPhase(mission_manager_->getPhase(),2);
               
            }
            else if (robot_state_->getTeleopCmdReceived()){
                mission_manager_->setPhase(mission_manager_->getPhase(),3);
                robot_state_->setReturnToMission(false);
                
            }
            
            break;
            }
        case 2:
            {   
                robot_state_->setTaskReferencesToZero();
                double distance_to_goal = robot_state_->getNormGoalDistance("goal_frame");
                //std::cout << "Distance to goal in stop phase: " << distance_to_goal << std::endl;
                if (distance_to_goal >= 0.0001 || robot_state_->getReturnToMission()){
                    mission_manager_->setPhase(mission_manager_->getPhase(),1);
                }
                else if (robot_state_->getTeleopCmdReceived()){
                    mission_manager_->setPhase(mission_manager_->getPhase(),3);
                    robot_state_->setReturnToMission(false);
                    
                }
                
            //std::cout << "In stop phase" << std::endl;
            // Do nothing, mission accomplished
            break; 
            }
        case 3:
            {
            double distance_to_teleop_goal = robot_state_->getNormGoalDistance("teleop");
            std::cout << "Distance to teleop goal: " << distance_to_teleop_goal << std::endl;
            if (robot_state_->getReturnToMission())
                {
                //std::cout << "Returning to mission from teleop" << std::endl;
                //std::cout << "Previous phase: " << mission_manager_->GetPreviousPhase() << std::endl;
                mission_manager_->setPhase(mission_manager_->getPhase(),mission_manager_->getPreviousPhase());
                robot_state_->setTeleopCmdReceived(false);
                
                }
            else if (!robot_state_->getTeleopCmdReceived())
                {
                mission_manager_->setPhase(mission_manager_->getPhase(),2);
                }
            else if (distance_to_teleop_goal < 0.0001 ){
                mission_manager_->setPhase(mission_manager_->getPhase(),2);
                robot_state_->setTeleopCmdReceived(false);
            }
            break;
            }
        }
    }
    Eigen::VectorXd getYdotbarResult(){
        return icat_result_.ydotbar;
    }
     void sendCommands(){
        // compute control
        // allocate joint velocities
        Eigen::VectorXd q_dot(7);
        // compute joint velocities
        q_dot = getYdotbarResult();
        // set joint velocities
        for (size_t i = 0; i < velocity_joint_handles_.size(); ++i) {
            velocity_joint_handles_[i].setCommand(q_dot(i));
        }
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
        robot_state_ = std::make_unique<franka_state>(node_handle_, *mission_manager_);
        
        return robot_state_->init(robot_hardware/*, arm_id */);
    };
    void starting(){
        // initialize mission manager
        mission_manager_->starting();
        robot_state_->starting();
    }
    void update(){
        robot_state_->update();
        updateTaskPriority( );
        robot_state_->updateState();
        sendCommands();
        updateMissionPhase();

    }
    
  
  

};
#endif
