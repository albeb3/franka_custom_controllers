#ifndef FRANKA_DATA_H
#define FRANKA_DATA_H
#include <Eigen/Dense>
#include <std_msgs/String.h>
#include <array>
#include <string>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include "franka_custom_controllers/mission_manager.hpp"
#include "franka_custom_controllers/my_utils.hpp"

#include "robot_perception/ProximityTaskArray.h"
#include "robot_perception/ProximityTask.h"

enum class Task_name{
    GOAL_MOVE,
    TELEOP,
    JOINT_LIMIT,
    JOINT_4_OTHM,
    JOINT_5_OTHM,
    JOINT_6_OTHM,
    JOINT_7_OTHM,
    JOINT_4_SELF,
    JOINT_5_SELF,
    JOINT_6_SELF,
    JOINT_7_SELF
};
constexpr std::array<franka::Frame, 5> Joint_frames_ = {
    franka::Frame::kJoint4,
    franka::Frame::kJoint5,
    franka::Frame::kJoint6,
    franka::Frame::kJoint7, 
    franka::Frame::kEndEffector
};
std::vector<std::string> link_names_ = {
    "_link4",
    "_link5",
    "_link6",
    "_link7"
};

struct Trasform_matrices{
    // Transformation matrices
    Eigen::Matrix<double, 4,4> b_T_e,b_T_g,b_T_gteleop;
    std::array<Eigen::Matrix<double,4,4>,4> b_T_i;
};

struct Jacobians{
    // Jacobian of goal position projected on vehicle frame
    Eigen::Matrix<double, 6, 7> Jee ;
    Eigen::Matrix<double, 7, 7> Jjl;
    Eigen::Matrix<double, 6, 7> Jteleop;
    std::array<Eigen::Matrix<double,6,7>,4> J;
    std::array<Eigen::Matrix<double,6,7>,4> J_self;
    std::array<Eigen::Matrix<double,6,7>,4> J_othM;
};
struct Action_transitions{
    Eigen::Matrix<double, 6,6> A_ee;
    Eigen::Matrix<double, 7,7> A_jl;
    Eigen::Matrix<double, 6,6> A_teleop;
    std::array<Eigen::Matrix<double, 6,6>,4> A_self;
    std::array<Eigen::Matrix<double, 6,6>,4> A_othM;
};
struct Task_references{
    Eigen::Matrix<double, 6,1> xdot_g_pos;
    Eigen::Matrix<double, 7,1> xdot_jl;
    Eigen::Matrix<double, 6,1> xdot_teleop;
    std::array<Eigen::Matrix<double, 6,1>,4> xdot_self;
    std::array<Eigen::Matrix<double, 6,1>,4> xdot_othM;
};
struct robot_state{
    // franka joint position 
    Eigen::Matrix<double, 7,1> q;
    // joint velocities
    Eigen::Matrix<double, 7,1> q_dot;
    // joint limits 
    const Eigen::Matrix<double, 7,1> joint_lower_limits = (Eigen::Matrix<double,7,1>() << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973).finished();
    const Eigen::Matrix<double, 7,1> joint_upper_limits = (Eigen::Matrix<double,7,1>() << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973).finished();
    //
    Trasform_matrices trasform_matrices;
    Action_transitions action_transitions;
    Jacobians jacobians;
    Task_references task_references;
    // 
};
class franka_state{

    private:
        // Node handle
        ros::NodeHandle& nh_;
        // TF listener
        tf::TransformListener& tf_listener_;
        // Franka handles
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        // Mission manager
        MissionManager& mission_manager_;
        // Robot state
        robot_state robot_state_;
        // Arm id
        std::string arm_id_;
        // Subscribers
        ros::Subscriber teleop_cmd_sub_;
        ros::Subscriber return_to_mission_sub_;
        ros::Subscriber self_proximity_task_sub_;
        ros::Subscriber other_proximity_task_sub_;
        // teleop variables
        bool teleop_cmd_received_= false;
        bool return_to_mission_= false;
        // proximity task variables
        // self min distances and points and directions and Ji_p points
        std::array<float,4> self_min_distances_;
        std::array<Eigen::Vector3d,4> self_min_points_;
        std::array<Eigen::Vector3d,4> self_direct_points_;
        std::array<Eigen::Vector3d,4> Ji_p_points_self_;
        // other min distances and points and directions and Ji_p points
        std::array<float,4> other_min_distances_;
        std::array<Eigen::Vector3d,4> other_min_points_;
        std::array<Eigen::Vector3d,4> other_direct_points_;
        std::array<Eigen::Vector3d,4> Ji_p_points_other_;
        // thresholds for proximity tasks
        float th_min_self_avoidance_;
        float th_max_self_avoidance_;
        float th_min_obstacle_avoidance_;
        float th_max_obstacle_avoidance_;
        float th_min_other_avoidance_;
        float th_max_other_avoidance_;
        // gain for task references
        float gain_teleop_;
        float gain_obstacle_avoidance_;
        float gain_self_avoidance_;
        float gain_other_avoidance_;
        float saturation_velocity_;

        // teleop callbacks
        void teleopCmdCallback(const std_msgs::Bool::ConstPtr& msg){
            //std::cout << "Teleop command received: " << msg->data << std::endl;
            teleop_cmd_received_ = msg->data;
        };
        void returnToMissionCallback(const std_msgs::Bool::ConstPtr& msg){
            
            return_to_mission_ = msg->data;
            //std::cout << "Return to mission command received: " << return_to_mission_ << std::endl;
        };
        // proximity task callbacks
        void selfProximityTaskCallback(const robot_perception::ProximityTaskArray::ConstPtr& msg) { 
            setSelfDistancesToZero();
            for (const auto& task : msg->proximity_tasks) {
                size_t idx = 0;
                for (; idx < link_names_.size(); ++idx) {
                    if (task.link_id == arm_id_ + link_names_[idx]) {
                        break;
                    }
                }
                if (idx < link_names_.size()) {
                    if (task.distance < self_min_distances_[idx]){
                        self_min_distances_[idx] = task.distance;
                        self_min_points_[idx] = Eigen::Vector3d(task.point.x, task.point.y, task.point.z); 
                        self_direct_points_[idx] = Eigen::Vector3d(task.direction.x, task.direction.y, task.direction.z); 
                    }
                }
            }
            /*
            for (const auto& task : msg->proximity_tasks) {
                if (task.link_id==arm_id_+"_link4"){ 
                    if (task.distance < self_min_distances_[0]){ 
                        self_min_distances_[0] = task.distance;
                        self_min_points_[0] = Eigen::Vector3d(task.point.x, task.point.y, task.point.z); 
                        self_direct_points_[0] = Eigen::Vector3d(task.direction.x, task.direction.y, task.direction.z); 
                        //std::cout << "Self min distance link 4: "<< self_min_distances_[0] << std::endl; 
                    }
                } 
                else if(task.link_id==arm_id_+"_link5"){ 
                    if (task.distance < self_min_distances_[1]){
                        self_min_distances_[1] = task.distance;
                        self_min_points_[1] = Eigen::Vector3d(task.point.x, task.point.y, task.point.z);
                        self_direct_points_[1] = Eigen::Vector3d(task.direction.x, task.direction.y, task.direction.z); 
                        //std::cout << "Self min distance link 5: "<< self_min_distances_[1] << std::endl; 
                    } 
                }
                else if(task.link_id==arm_id_+"_link6"){ 
                    if (task.distance < self_min_distances_[2]){ 
                        self_min_distances_[2] = task.distance;
                        self_min_points_[2] = Eigen::Vector3d(task.point.x, task.point.y, task.point.z); 
                        self_direct_points_[2] = Eigen::Vector3d(task.direction.x, task.direction.y, task.direction.z); 
                        //std::cout << "Self min distance link 6: "<< self_min_distances_[2] << std::endl; 
                    } 
                } 
                else if(task.link_id==arm_id_+"_link7"){ 
                    if (task.distance < self_min_distances_[3]){
                        self_min_distances_[3] = task.distance;
                        self_min_points_[3] = Eigen::Vector3d(task.point.x, task.point.y, task.point.z); 
                        self_direct_points_[3] = Eigen::Vector3d(task.direction.x, task.direction.y, task.direction.z); 
                        //std::cout << "Self min distance link 7: "<< self_min_distances_[3] << std::endl; 
                    } 
                } 
            }
             */
            for (size_t i=0; i< self_min_distances_.size(); i++){
                if(self_min_distances_[i]<th_max_self_avoidance_){
                    //std::cout << "Self min distance link "<< i+4 << ": "<< self_min_distances_[i] << std::endl;
                }
            }
                
        };
        void otherProximityTaskCallback(const robot_perception::ProximityTaskArray::ConstPtr& msg) {
            setOthMDistancesToZero();
            for (const auto& task : msg->proximity_tasks) {
                size_t idx = 0;
                for (; idx < link_names_.size(); ++idx) {
                    if (task.link_id == arm_id_ + link_names_[idx]) {
                        break;
                    }
                }
                if (idx < link_names_.size()) {
                    if (task.distance < other_min_distances_[idx]){
                        other_min_distances_[idx] = task.distance;
                        other_min_points_[idx] = Eigen::Vector3d(task.point.x, task.point.y, task.point.z); 
                        other_direct_points_[idx] = Eigen::Vector3d(task.direction.x, task.direction.y, task.direction.z); 
                    }
                }
            }
            /*
             for (const auto& task : msg->proximity_tasks) {
                if (task.link_id==arm_id_+"_link4"){ 
                    if (task.distance < other_min_distances_[0]){ 
                        other_min_distances_[0] = task.distance; 
                        other_min_points_[0] = Eigen::Vector3d(task.point.x, task.point.y, task.point.z); 
                        other_direct_points_[0] = Eigen::Vector3d(task.direction.x, task.direction.y, task.direction.z); 
                        //std::cout << "Other min distance link 4: "<< other_min_distances_[0] << std::endl; 
                    } 
                } 
                else if(task.link_id==arm_id_+"_link5"){ 
                    if (task.distance < other_min_distances_[1]){ 
                        other_min_distances_[1] = task.distance; 
                        other_min_points_[1] = Eigen::Vector3d(task.point.x, task.point.y, task.point.z);
                        other_direct_points_[1] = Eigen::Vector3d(task.direction.x, task.direction.y, task.direction.z); 
                        //std::cout << "Other min distance link 5: "<< other_min_distances_[1] << std::endl; 
                    } 
                }
                else if(task.link_id==arm_id_+"_link6"){ 
                    if (task.distance < other_min_distances_[2]){ 
                        other_min_distances_[2] = task.distance; 
                        other_min_points_[2] = Eigen::Vector3d(task.point.x, task.point.y, task.point.z); 
                        other_direct_points_[2] = Eigen::Vector3d(task.direction.x, task.direction.y, task.direction.z); 
                        //std::cout << "Other min distance link 6: "<< other_min_distances_[2] << std::endl; 
                    } 
                } 
                else if(task.link_id==arm_id_+"_link7"){ 
                    if (task.distance < other_min_distances_[3]){
                        other_min_distances_[3] = task.distance; 
                        other_min_points_[3] = Eigen::Vector3d(task.point.x, task.point.y, task.point.z); 
                        other_direct_points_[3] = Eigen::Vector3d(task.direction.x, task.direction.y, task.direction.z);
                        //std::cout << "Other min distance link 7: "<< other_min_distances_[3] << std::endl;
                        }
                    }
                }
            */
           
            for (size_t i=0; i< other_min_distances_.size(); i++){
                if(other_min_distances_[i]<th_max_other_avoidance_){
                    std::cout << "Other min distance link "<< i+4 << ": "<< other_min_distances_[i] << std::endl;
                }
                
            }
        };
        // 
        void computeJacobian(){
            // set joint limit jacobian as identity
            robot_state_.jacobians.Jjl = Eigen::Matrix<double,7,7>::Identity(); 

            // set link jacobians
            for (size_t i=0; i<Joint_frames_.size()-1; i++){ // for each joint less EE
                
                // set jacobian of joint i if distance to collision model < 0.1 m
                if  (self_min_distances_[i]<th_max_self_avoidance_ || other_min_distances_[i]<th_max_other_avoidance_){
                    std::array<double,42> jacobian_array= model_handle_->getZeroJacobian(Joint_frames_[i]);
                    Eigen::Map<Eigen::Matrix<double,6,7>> jacobian(jacobian_array.data());
                    // set jacobian of joint i
                    robot_state_.jacobians.J[i] = jacobian;
                    if (self_min_distances_[i]<th_max_self_avoidance_){
                        // set jacobian of closest point on self collision model
                        robot_state_.jacobians.J_self[i] = computeRigidBodyJacobian(Ji_p_points_self_[i])*jacobian;
                    }
                    if (other_min_distances_[i]<th_max_other_avoidance_){
                        // set jacobian of closest point on other robot collision model
                        robot_state_.jacobians.J_othM[i] = computeRigidBodyJacobian(Ji_p_points_other_[i])*jacobian;
                    }
                }
                else{
                    robot_state_.jacobians.J[i] = Eigen::Matrix<double,6,7>::Zero();
                    robot_state_.jacobians.J_self[i] = Eigen::Matrix<double,6,7>::Zero();
                    robot_state_.jacobians.J_othM[i] = Eigen::Matrix<double,6,7>::Zero();
                }
            }

            // set end-effector jacobian
            std::array<double,42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
            Eigen::Map<Eigen::Matrix<double,6,7>> jacobian(jacobian_array.data());
            robot_state_.jacobians.Jee = jacobian;
            // set teleop jacobian as end-effector jacobian
            robot_state_.jacobians.Jteleop = jacobian;
            /*
            std::array<double, 42> jacobian_4_array = model_handle_->getZeroJacobian(franka::Frame::kJoint4);
            Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_4(jacobian_4_array.data());
            robot_state_.jacobians.J[0] = jacobian_4;
            robot_state_.jacobians.J_self[0] = ComputeRigidBodyJacobian(Ji_p_points_self_[0])*jacobian_4;
            robot_state_.jacobians.J_othM[0] = ComputeRigidBodyJacobian(Ji_p_points_other_[0])*jacobian_4;
            std::array<double, 42> jacobian_5_array = model_handle_->getZeroJacobian(franka::Frame::kJoint5);
            Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_5(jacobian_5_array.data());
            robot_state_.jacobians.J[1] = jacobian_5;
            robot_state_.jacobians.J_self[1] = ComputeRigidBodyJacobian(Ji_p_points_self_[1])*jacobian_5;
            robot_state_.jacobians.J_othM[1] = ComputeRigidBodyJacobian(Ji_p_points_other_[1])*jacobian_5;
            std::array<double, 42> jacobian_6_array = model_handle_->getZeroJacobian(franka::Frame::kJoint6);
            Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_6(jacobian_6_array.data());
            robot_state_.jacobians.J[2] = jacobian_6;
            robot_state_.jacobians.J_self[2] = ComputeRigidBodyJacobian(Ji_p_points_self_[2])*jacobian_6;
            robot_state_.jacobians.J_othM[2] = ComputeRigidBodyJacobian(Ji_p_points_other_[2])*jacobian_6;
            std::array<double, 42> jacobian_7_array = model_handle_->getZeroJacobian(franka::Frame::kJoint7);
            Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_7(jacobian_7_array.data());
            robot_state_.jacobians.J[3] = jacobian_7;
            robot_state_.jacobians.J_self[3] = ComputeRigidBodyJacobian(Ji_p_points_self_[3])*jacobian_7;
            robot_state_.jacobians.J_othM[3] = ComputeRigidBodyJacobian(Ji_p_points_other_[3])*jacobian_7;
            std::array<double, 42> jacobian_EE_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
            Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_EE(jacobian_EE_array.data());
            robot_state_.jacobians.Jee = jacobian_EE;
            // set teleop jacobian as end-effector jacobian
            robot_state_.jacobians.Jteleop = jacobian_EE;
            */
            
        };
        void updateTransform(){
            // update end-effector transform
            franka::RobotState robot_state = state_handle_->getRobotState();
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            robot_state_.trasform_matrices.b_T_e = transform.matrix();
            
            // update link transforms
            const std::string arm_id_link = arm_id_ + "_link";
            tf::StampedTransform tf;
            Eigen::Affine3d eigen_tf;
            // update link 4 transform
            for (size_t i=0; i<4; i++){
                try{
                    tf_listener_.lookupTransform(arm_id_link + "0", arm_id_link + std::to_string(i+4), ros::Time(0), tf);
                    tf::transformTFToEigen(tf, eigen_tf);
                    robot_state_.trasform_matrices.b_T_i[i] = eigen_tf.matrix();
                    Ji_p_points_self_[i] = self_min_points_[i] - eigen_tf.matrix().block<3,1>(0,3);
                    Ji_p_points_other_[i] = other_min_points_[i] - eigen_tf.matrix().block<3,1>(0,3);
                }
                catch (tf::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    continue;
                }
            }  

            // update goal transform if exists3
            if(getGoalFrameExists("goal_frame")){
                try{
                    tf_listener_.lookupTransform(arm_id_link + "0", "goal_frame_" + arm_id_, ros::Time(0), tf);
                    tf::transformTFToEigen(tf, eigen_tf);
                    robot_state_.trasform_matrices.b_T_g =  eigen_tf.matrix();
                }
                catch (tf::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    return;
                }
            }
            // update teleop goal transform if exists
            if(getGoalFrameExists(arm_id_+"_teleop_frame")){
                try{
                    tf_listener_.lookupTransform(arm_id_link + "0", arm_id_+"_teleop_frame", ros::Time(0), tf);
                    tf::transformTFToEigen(tf, eigen_tf);
                    robot_state_.trasform_matrices.b_T_gteleop =  eigen_tf.matrix();
                }
                catch (tf::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    return;
                }
            }
        };
        void computeTaskReference(){
            // compute joint limit avoidance velocities
            for (size_t i=0; i<7; i++){
                double mean_joint_position = (robot_state_.joint_lower_limits(i) + robot_state_.joint_upper_limits(i)) / 2.0;
                if (robot_state_.q(i) < mean_joint_position){
                    robot_state_.task_references.xdot_jl(i) = 0.2* (robot_state_.joint_lower_limits(i)-robot_state_.q(i) + 0.01) ;
                }else{
                    robot_state_.task_references.xdot_jl(i) = 0.2* (robot_state_.joint_upper_limits(i)-robot_state_.q(i) - 0.01) ;
                }
            }
            
            // compute self and other manipulator proximity task references
            for (size_t i=0; i<4; i++){
                robot_state_.task_references.xdot_self[i].setZero();
                robot_state_.task_references.xdot_othM[i].setZero();
                // self proximity task references
                if (self_min_distances_[i]<th_max_self_avoidance_){
                    Eigen::Vector3d xdot;
                    //xdot = -self_direct_points_[i] / (self_min_distances_[i] + 0.3);
                    xdot = -self_direct_points_[i] * gain_self_avoidance_;
                    //robot_state_.task_references.xdot_self[i].head(3) = 0.9 * xdot;
                    robot_state_.task_references.xdot_self[i].head(3) = xdot;
                }
                // other manipulator proximity task references
                if (other_min_distances_[i]<th_max_other_avoidance_){
                    Eigen::Vector3d xdot;
                    //xdot = -other_direct_points_[i] / (other_min_distances_[i] + 0.3);
                    if (other_min_distances_[i]-th_min_other_avoidance_ >=0.0){
                         xdot = -other_direct_points_[i] * gain_other_avoidance_ * (other_min_distances_[i]-th_min_other_avoidance_);
                    }
                    else {
                         xdot = -other_direct_points_[i] * gain_other_avoidance_;
                    }
                    //robot_state_.task_references.xdot_othM[i].head(3) = 0.9 * xdot;
                    robot_state_.task_references.xdot_othM[i].head(3) = xdot;
                }
                //std::cout << "Self task xdot link "<< i+4 << ": "<< robot_state_.task_references.xdot_self[i].head(3).transpose() << std::endl;
               //std::cout << "Gain other manipulator avoidance link "<< i+4 << ": "<< gain_other_avoidance_ << std::endl;
                //std::cout << "Other task xdot link "<< i+4 << ": "<< robot_state_.task_references.xdot_othM[i].head(3).transpose() << std::endl;
            }

            // compute goal position velocity if goal frame exists
            switch (mission_manager_.getPhase())
            {
            case 1: // reaching goal phase
                // compute cartesian error for goal frame position
                robot_state_.task_references.xdot_g_pos = 
                    computeCartesianTask(robot_state_.trasform_matrices.b_T_g, robot_state_.trasform_matrices.b_T_e, gain_teleop_);
                break;
            case 2: // stop phase
                robot_state_.task_references.xdot_g_pos.setZero();
                break; 
            case 3: // teleop phase
                robot_state_.task_references.xdot_teleop = 
                    computeCartesianTask(robot_state_.trasform_matrices.b_T_gteleop, robot_state_.trasform_matrices.b_T_e, gain_teleop_);
                break;
            default:
                break;
            }
        };
        void computeActivationFunction(){
            // Compute JOINT LIMIT activation matrix
            robot_state_.action_transitions.A_jl=Eigen::Matrix<double, 7, 7>::Identity();
            for (size_t i=0; i<7;i++){
                robot_state_.action_transitions.A_jl(i,i)=increasingBellShapedFunction(0.9*robot_state_.joint_upper_limits(i), robot_state_.joint_upper_limits(i),
                                                                                    0.0, 1.0, robot_state_.q(i)) +
                                                        decreasingBellShapedFunction(robot_state_.joint_lower_limits(i), 0.9*robot_state_.joint_lower_limits(i),
                                                                                    0.0, 1.0, robot_state_.q(i));
            }
            // Compute AVOIDANCE activation matrices
            for (size_t i=0; i<4; i++){
                robot_state_.action_transitions.A_self[i]=Eigen::Matrix<double, 6, 6>::Zero();
                for (size_t j=0; j<3; j++){
                    robot_state_.action_transitions.A_self[i](j,j)= decreasingBellShapedFunction(th_min_self_avoidance_, th_max_self_avoidance_, 0.0, 1.0, self_min_distances_[i]);
                    robot_state_.action_transitions.A_othM[i](j,j)= decreasingBellShapedFunction(th_min_other_avoidance_, th_max_other_avoidance_, 0.0, 1.0, other_min_distances_[i]);
                    
                }
            }
            
            mission_manager_.updateMissionData();
            std::vector<std::string> prev_action = mission_manager_.getPrevAction();
            std::vector<std::string> curr_action = mission_manager_.getCurrAction();
            double phase_time = mission_manager_.getPhaseTime();
            // Compute GOAL REACHING activation matrix
            robot_state_.action_transitions.A_ee=Eigen::Matrix<double, 6, 6>::Identity();
            robot_state_.action_transitions.A_ee *= mission_manager_.actionTransition("R_M", prev_action, curr_action, phase_time);
            // Compute TELEOPERATION activation matrix
            robot_state_.action_transitions.A_teleop=Eigen::Matrix<double, 6, 6>::Identity();
            robot_state_.action_transitions.A_teleop *= mission_manager_.actionTransition("T_M", prev_action, curr_action, phase_time);
        }
        // Helper functions
        Eigen::Matrix<double, 6, 1> computeCartesianTask(const Eigen::Matrix4d& T_g, const Eigen::Matrix4d& T_e, const double gain){
            CartErrorResult cart_error = cartError( T_g,T_e);
            Eigen::Matrix<double,3,1> pos_error = cart_error.pos_error/cart_error.pos_error.norm();
            Eigen::Matrix<double,3,1> ori_error = cart_error.ori_error/cart_error.ori_error.norm();
            Eigen::Matrix<double, 6, 1> x_dot;
            x_dot.head(3) <<  gain * pos_error;
            x_dot.tail(3) <<  gain * ori_error;
            return saturate( x_dot, saturation_velocity_);
        }
        void setSelfDistancesToZero(){
            for (size_t i=0; i<Joint_frames_.size()-1; i++){    // for each joint less EE
                self_min_distances_[i] = std::numeric_limits<float>::max();
                self_min_points_[i] = Eigen::Vector3d::Zero();
                self_direct_points_[i] = Eigen::Vector3d::Zero();
            }
        };
        void setOthMDistancesToZero(){
            for (size_t i=0; i<Joint_frames_.size()-1; i++){    // for each joint less EE
                other_min_distances_[i] = std::numeric_limits<float>::max();
                other_min_points_[i] = Eigen::Vector3d::Zero();
                other_direct_points_[i] = Eigen::Vector3d::Zero();
            }
        };
       
    public:
        franka_state(ros::NodeHandle& nh, MissionManager& mission_manager, tf::TransformListener& tf_listener): nh_(nh), mission_manager_(mission_manager), tf_listener_(tf_listener){
            teleop_cmd_sub_ = nh_.subscribe<std_msgs::Bool>("teleop_cmd_received", 1, &franka_state::teleopCmdCallback, this);
            return_to_mission_sub_ = nh_.subscribe<std_msgs::Bool>("return_to_mission", 1, &franka_state::returnToMissionCallback, this);
            self_proximity_task_sub_ = nh_.subscribe<robot_perception::ProximityTaskArray>("proximity_tasks", 1, &franka_state::selfProximityTaskCallback, this);
            other_proximity_task_sub_ = nh_.subscribe<robot_perception::ProximityTaskArray>("proximity_tasks_other_manipulator", 1, &franka_state::otherProximityTaskCallback, this);
        };
        ~franka_state()= default;
        bool init(hardware_interface::RobotHW* robot_hardware/*, std::string arm_id*/){
            //arm_id_ = arm_id;
            // Get parameters from parameter server
            // arm id--------------------------------------------------------------------------------------
            std::string arm_id;
            if (!nh_.getParam("arm_id", arm_id)) {
                ROS_ERROR("JointVelocityController: Could not get parameter arm_id");
                return false;
            }
            arm_id_ = arm_id;
            ROS_WARN_STREAM("JointVelocityController: arm_id set to " << arm_id_);
            // thresholds for proximity tasks----------------------------------------------------------------
            // -------------------------------MIN-------------------------------------------------------------------
            float th_min_self_avoidance;
            if (!nh_.getParam("th_min_self_avoidance", th_min_self_avoidance)) {
                ROS_ERROR("JointVelocityController: Could not get parameter th_min_self_avoidance");
                return false;
            }
            ROS_WARN_STREAM("JointVelocityController: th_min_self_avoidance set to " << th_min_self_avoidance);
            th_min_self_avoidance_ = th_min_self_avoidance;
            
            float th_min_other_avoidance;
            if (!nh_.getParam("th_min_other_avoidance", th_min_other_avoidance)) {
                ROS_ERROR("JointVelocityController: Could not get parameter th_min_other_avoidance");
                return false;
            }
            ROS_WARN_STREAM("JointVelocityController: th_min_other_avoidance set to " << th_min_other_avoidance);
            th_min_other_avoidance_ = th_min_other_avoidance;
            
            float th_min_obstacle_avoidance;
            if (!nh_.getParam("th_min_obstacle_avoidance", th_min_obstacle_avoidance)) {
                ROS_ERROR("JointVelocityController: Could not get parameter th_min_obstacle_avoidance");
                return false;
            }
            th_min_obstacle_avoidance_ = th_min_obstacle_avoidance;
            ROS_WARN_STREAM("JointVelocityController: th_min_obstacle_avoidance set to " << th_min_obstacle_avoidance_);
            // -------------------------------MAX--------------------------------------------------------------------
            float th_max_other_avoidance;
            if( !nh_.getParam("th_max_other_avoidance", th_max_other_avoidance)) {
                ROS_ERROR("JointVelocityController: Could not get parameter th_max_other_avoidance");
                return false;
            }
            th_max_other_avoidance_ = th_max_other_avoidance;
            ROS_WARN_STREAM("JointVelocityController: th_max_other_avoidance set to " << th_max_other_avoidance);
            
            float th_max_self_avoidance;
            if (!nh_.getParam("th_max_self_avoidance", th_max_self_avoidance)) {
                ROS_ERROR("JointVelocityController: Could not get parameter th_max_self_avoidance");
                return false;
            }
            ROS_WARN_STREAM("JointVelocityController: th_max_self_avoidance set to " << th_max_self_avoidance);
            th_max_self_avoidance_ = th_max_self_avoidance;
            
            float th_max_obstacle_avoidance;
            if (!nh_.getParam("th_max_obstacle_avoidance", th_max_obstacle_avoidance)) {
                ROS_ERROR("JointVelocityController: Could not get parameter th_max_obstacle_avoidance");
                return false;
            }
            ROS_WARN_STREAM("JointVelocityController: th_max_obstacle_avoidance set to " << th_max_obstacle_avoidance);
            th_max_obstacle_avoidance_ = th_max_obstacle_avoidance;
            // gains for task references--------------------------------------------------------------------
            //-------------------------------AVOIDANCE-------------------------------------------------------
            float gain_other_avoidance;
            if( !nh_.getParam("gain_other_avoidance", gain_other_avoidance)) {
                ROS_ERROR("JointVelocityController: Could not get parameter gain_other_avoidance");
                return false;
            }
            gain_other_avoidance_ = gain_other_avoidance;
            ROS_WARN_STREAM("JointVelocityController: gain_other_avoidance set to " << gain_other_avoidance_);

            double gain_self_avoidance;
            if( !nh_.getParam("gain_self_avoidance", gain_self_avoidance)) {
                ROS_ERROR("JointVelocityController: Could not get parameter gain_self_avoidance");
                return false;
            }
            gain_self_avoidance_ = gain_self_avoidance;
            ROS_WARN_STREAM("JointVelocityController: gain_self_avoidance set to " << gain_self_avoidance_);
            
            double gain_obstacle_avoidance;
            if( !nh_.getParam("gain_obstacle_avoidance", gain_obstacle_avoidance)) {
                ROS_ERROR("JointVelocityController: Could not get parameter gain_obstacle_avoidance");
                return false;
            }
            gain_obstacle_avoidance_ = gain_obstacle_avoidance;
            ROS_WARN_STREAM("JointVelocityController: gain_obstacle_avoidance set to " << gain_obstacle_avoidance_);
            // -------------------------------GOAL REACHING and TELEOP-----------------------------------------
            double gain_teleop;
            if( !nh_.getParam("gain_teleop", gain_teleop)) {
                ROS_ERROR("JointVelocityController: Could not get parameter gain_teleop");
                return false;
            }
            gain_teleop_ = gain_teleop;
            ROS_WARN_STREAM("JointVelocityController: gain_teleop set to " << gain_teleop_);
            // saturation velocity for cartesian tasks------------------------------------------------------
            double saturation_velocity;
            if (!nh_.getParam("saturation_velocity", saturation_velocity)) {
                ROS_ERROR("JointVelocityController: Could not get parameter saturation_velocity");
                return false;
            }
            ROS_WARN_STREAM("JointVelocityController: saturation_velocity set to " << saturation_velocity);
            saturation_velocity_ = saturation_velocity;
            // END PARAMETERS GETTING-------------------------------------------------------------------------
            // Get handles---------------------------------------------------------------------------------
            auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
            if (model_interface == nullptr) {
                ROS_ERROR_STREAM(
                    "JointVelocityController: Error getting model interface from hardware");
                return false;
            }
            try {
                model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                    model_interface->getHandle(arm_id + "_model"));
            } catch (hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM(
                    "JointVelocityController: Exception getting model handle from interface: "
                    << ex.what());
                return false;
            }
            auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
            if (state_interface == nullptr) {
                ROS_ERROR("JointVelocityController: Could not get state interface from hardware");
                return false;
            }
            try {
                state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                    state_interface->getHandle(arm_id + "_robot"));
            } catch (hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM(
                    "JointVelocityController: Exception getting state handle from interface: "
                    << ex.what());
                return false;
            }
            return true;
        };
        void starting(){
            franka::RobotState initial_state = state_handle_->getRobotState();
            // convert to eigen
            Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
            Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
            robot_state_.q = q_initial;
            updateTransform();
            setTaskReferencesToZero();
            computeJacobian();
            setSelfDistancesToZero();
            setOthMDistancesToZero();
        }    
        void update(){
            updateTransform();
            computeJacobian();
            computeActivationFunction();
            computeTaskReference();
            updateState();
        };
        void updateState(){
            for (size_t i=0; i<7; i++){
                robot_state_.q(i) = state_handle_->getRobotState().q[i];
            }
        };
         void setTaskReferencesToZero(){
            robot_state_.task_references.xdot_g_pos = Eigen::Matrix<double,6,1>::Zero();
            robot_state_.task_references.xdot_jl = Eigen::Matrix<double,7,1>::Zero();
            robot_state_.task_references.xdot_teleop = Eigen::Matrix<double,6,1>::Zero();
            for( size_t i=0; i<Joint_frames_.size()-1; i++){   // for each joint less EE
                robot_state_.task_references.xdot_self[i] = Eigen::Matrix<double,6,1>::Zero();
                robot_state_.task_references.xdot_othM[i] = Eigen::Matrix<double,6,1>::Zero();
            }
        };
        
        bool getGoalFrameExists(const std::string& goal){
            if (goal == "goal_frame"){
                return tf_listener_.frameExists("goal_frame_" + arm_id_);
            }
            else if (goal == arm_id_ +"_teleop_frame"){
                return tf_listener_.frameExists(arm_id_ +"_teleop_frame");
            }
            else{
                return false;
            }
        };
        double getNormGoalDistance(std::string goal){
            if (goal == "goal_frame"){
                CartErrorResult cart_error = cartError( robot_state_.trasform_matrices.b_T_g,robot_state_.trasform_matrices.b_T_e);
                Eigen::Matrix<double,6,1> goal_distance;
                goal_distance.head<3>() = cart_error.pos_error;
                goal_distance.tail<3>() = cart_error.ori_error;
                return goal_distance.squaredNorm();
                
                }
            else if (goal == "teleop"){
                CartErrorResult cart_error = cartError( robot_state_.trasform_matrices.b_T_gteleop,robot_state_.trasform_matrices.b_T_e);
                Eigen::Matrix<double,6,1> goal_distance;
                goal_distance.head<3>() = cart_error.pos_error;
                goal_distance.tail<3>() = cart_error.ori_error;
                return goal_distance.squaredNorm();
            }
            else{
                Eigen::Matrix<double,6,1> goal_distance;
                goal_distance << 0.0,0.0,0.0,0.0,0.0,0.0;
                return goal_distance.squaredNorm();
            }
        };
        Eigen::MatrixXd getJacobian( Task_name task_name){
            switch (task_name)
            {
            case Task_name::GOAL_MOVE: return robot_state_.jacobians.Jee;
            case Task_name::JOINT_LIMIT: return robot_state_.jacobians.Jjl;
            case Task_name::TELEOP: return robot_state_.jacobians.Jteleop;
            case Task_name::JOINT_4_SELF: return robot_state_.jacobians.J_self[0];
            case Task_name::JOINT_5_SELF: return robot_state_.jacobians.J_self[1];
            case Task_name::JOINT_6_SELF: return robot_state_.jacobians.J_self[2];
            case Task_name::JOINT_7_SELF: return robot_state_.jacobians.J_self[3];
            case Task_name::JOINT_4_OTHM: return robot_state_.jacobians.J_othM[0];  
            case Task_name::JOINT_5_OTHM: return robot_state_.jacobians.J_othM[1];
            case Task_name::JOINT_6_OTHM: return robot_state_.jacobians.J_othM[2];
            case Task_name::JOINT_7_OTHM: return robot_state_.jacobians.J_othM[3];
            default: return Eigen::MatrixXd::Zero(6,7);
            }
        };    
        Eigen::MatrixXd getActivationMatrix(Task_name  task_name){
            switch (task_name)
            {
            case Task_name::GOAL_MOVE: return robot_state_.action_transitions.A_ee;
            case Task_name::JOINT_LIMIT: return robot_state_.action_transitions.A_jl;
            case Task_name::TELEOP: return robot_state_.action_transitions.A_teleop;
            case Task_name::JOINT_4_SELF: return robot_state_.action_transitions.A_self[0];
            case Task_name::JOINT_5_SELF: return robot_state_.action_transitions.A_self[1];
            case Task_name::JOINT_6_SELF: return robot_state_.action_transitions.A_self[2];
            case Task_name::JOINT_7_SELF: return robot_state_.action_transitions.A_self[3];
            case Task_name::JOINT_4_OTHM: return robot_state_.action_transitions.A_othM[0];  
            case Task_name::JOINT_5_OTHM: return robot_state_.action_transitions.A_othM[1];
            case Task_name::JOINT_6_OTHM: return robot_state_.action_transitions.A_othM[2];
            case Task_name::JOINT_7_OTHM: return robot_state_.action_transitions.A_othM[3];
            default: return Eigen::MatrixXd::Zero(6,6);
            }
        };
        Eigen::MatrixXd getTaskReference(Task_name task_name){
            switch (task_name)
            {
            case Task_name::GOAL_MOVE: return robot_state_.task_references.xdot_g_pos;
            case Task_name::JOINT_LIMIT: return robot_state_.task_references.xdot_jl;
            case Task_name::TELEOP: return robot_state_.task_references.xdot_teleop;
            case Task_name::JOINT_4_SELF: return robot_state_.task_references.xdot_self[0];
            case Task_name::JOINT_5_SELF: return robot_state_.task_references.xdot_self[1];
            case Task_name::JOINT_6_SELF: return robot_state_.task_references.xdot_self[2];
            case Task_name::JOINT_7_SELF: return robot_state_.task_references.xdot_self[3];
            case Task_name::JOINT_4_OTHM: return robot_state_.task_references.xdot_othM[0];  
            case Task_name::JOINT_5_OTHM: return robot_state_.task_references.xdot_othM[1];
            case Task_name::JOINT_6_OTHM: return robot_state_.task_references.xdot_othM[2];
            case Task_name::JOINT_7_OTHM: return robot_state_.task_references.xdot_othM[3];
            default: return Eigen::MatrixXd::Zero(6,1);
            }
        };
    
        bool getReturnToMission(){
            return return_to_mission_;
        };
        bool getTeleopCmdReceived(){
            return teleop_cmd_received_;
        };
        void setReturnToMission(bool val){
            return_to_mission_ = val;
        };
        void setTeleopCmdReceived(bool val){
            teleop_cmd_received_ = val;
        };
        
};
#endif
