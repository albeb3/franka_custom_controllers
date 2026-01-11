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


struct Trasform_matrices{
    // Transformation matrices
    Eigen::Matrix<double, 4,4> w_T_b,w_T_e;
};

struct Jacobians{
    // Jacobian of goal position projected on vehicle frame
    Eigen::Matrix<double, 6, 7> Jee ;
    //Eigen::Matrix<double, 6, 7> Jteleop;
};
struct Action_transitions{
    Eigen::Matrix<double, 6,6> A_ee;
    //Eigen::Matrix<double, 6,6> A_teleop;
};
struct Task_references{
    Eigen::Matrix<double, 6,1> xdot_g_pos;
    //Eigen::Matrix<double, 6,1> xdot_teleop;
};
struct robot_state{
    // franka joint position 
    Eigen::Matrix<double, 7,1> q;
    // joint velocities
    Eigen::Matrix<double, 7,1> q_dot;
    Trasform_matrices trasform_matrices;
    Action_transitions action_transitions;
    Jacobians jacobians;
    Task_references task_references;
    // 
};
class franka_state{

    private:
    
    tf::TransformListener& tf_listener_;
    ros::NodeHandle& nh_;
    ros::Subscriber sub_goal_pose_ ;
    std::mutex position_and_orientation_d_target_mutex_;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    Eigen::Matrix<double,7,1> q_d_nullspace_;

    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond orientation_d_target_;

    
        
    
    //bool teleop_cmd_received_ = false;
    //bool return_to_mission_ = false;
    
   
    void computeJacobian(){

    };
    void updateTransform(){
        tf::StampedTransform tf;
        Eigen::Affine3d eigen_tf;
        if (tf_listener_.frameExists("world") && tf_listener_.frameExists("panda_R_link0")){
        
            tf_listener_.lookupTransform("world", "panda_R_link0", ros::Time(0), tf);
            tf::transformTFToEigen(tf, eigen_tf);
             robot_state_.trasform_matrices.w_T_b = eigen_tf.matrix() ;
        }
        if(tf_listener_.frameExists("panda_R_link0") && tf_listener_.frameExists("panda_R_linkEE")){
        
            tf_listener_.lookupTransform("panda_R_link0", "panda_R_linkEE", ros::Time(0), tf);
            tf::transformTFToEigen(tf, eigen_tf);
             robot_state_.trasform_matrices.w_T_e = eigen_tf.matrix() ;
        }
        
       

    };
    
    public:
    MissionManager& mission_manager_;
   robot_state robot_state_;
   bool goal_frame_exists_ = false;
    franka_state(ros::NodeHandle& nh,tf::TransformListener& tf_listener,MissionManager& mission_manager) 
                                        : nh_(nh), tf_listener_(tf_listener), mission_manager_(mission_manager) {
        
    };
    ~franka_state()= default;
    void Init(Eigen::Matrix<double,7,1> q_init, Eigen::Affine3d initial_transform, Eigen::Matrix<double,6,7> jacobian){
        // set equilibrium point to current state
        position_d_ = initial_transform.translation();
        orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
        position_d_target_ = initial_transform.translation();
        orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());
        // set initial transform
        robot_state_.trasform_matrices.w_T_e = initial_transform.matrix();
        // set initial jacobian
        robot_state_.jacobians.Jee= jacobian;
         // set nullspace equilibrium configuration to initial q
        q_d_nullspace_ = q_init ;

    };
    void updateState(){
        updateTransform();
        computeJacobian();
    };
    void setZeroJacobian(Eigen::Matrix<double,6,7> b_Jee){
        //refer jacobian to world frame throught trasformation matrix

        //robot_state_.jacobians.Jee.block<3,7>(0,0) = robot_state_.trasform_matrices.w_T_b.block<3,3>(0,0) * b_Jee.block<3,7>(0,0);
        //robot_state_.jacobians.Jee.block<3,7>(3,0) = robot_state_.trasform_matrices.w_T_b.block<3,3>(0,0) * b_Jee.block<3,7>(3,0);
        robot_state_.jacobians.Jee = b_Jee;
        
    };
    
    void computeTaskReference(){
        /*// update desired pose to target pose 
        std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
        position_d_ = position_d_target_;
        orientation_d_ = orientation_d_target_;
         if(tf_listener_.frameExists("goal_frame")){
            // position error
            Eigen::Matrix<double, 6, 1> error ;
            error.head(3) << robot_state_.b_p_e  - position_d_;
            // orientation error
            Eigen::Quaterniond current_orientation(robot_state_.b_rot_e);
            if (orientation_d_.coeffs().dot(current_orientation.coeffs()) < 0.0) {
                current_orientation.coeffs() << -current_orientation.coeffs();
            }
            // "difference" quaternion
            Eigen::Quaterniond error_quaternion(current_orientation.inverse() * orientation_d_);
            error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
            // Transform to base frame
            error.tail(3) << -current_orientation.toRotationMatrix() * error.tail(3);
            // compute desired velocity in Cartesian space
            Eigen::Matrix<double, 6, 1> x_dot;
            x_dot.head(3) <<  error.head(3);  
            x_dot.tail(3) <<  error.tail(3);
            robot_state_.task_references.xdot_g_pos = x_dot;
            }*/
        if(tf_listener_.frameExists("goal_frame")){
            Eigen::Matrix<double, 4,4> w_T_g,w_T_e;
            tf::StampedTransform tf;
            Eigen::Affine3d eigen_tf;
            try{
                tf_listener_.lookupTransform("panda_R_link0", "goal_frame", ros::Time(0), tf);
                tf::transformTFToEigen(tf, eigen_tf);
                w_T_g =  eigen_tf.matrix();
            }
            catch (tf::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                return;
            }
            try{
                tf_listener_.lookupTransform("panda_R_link0", "panda_R_EE", ros::Time(0), tf);
                tf::transformTFToEigen(tf, eigen_tf);
                w_T_e =  eigen_tf.matrix();
            }
            catch (tf::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                return;
            }
            CartErrorResult cart_error = CartError(w_T_g,w_T_e );
            
            Eigen::Matrix<double, 6, 1> error ;
            // orientation error
            error.head(3) <<  cart_error.ori_error ;
            // position error
            error.tail(3) <<  cart_error.pos_error ;
            std::cout << "EE pose: \n" << w_T_e.block<3,1>(0,3) << std::endl;
            std::cout << "Goal pose: \n" << w_T_g.block<3,1>(0,3) << std::endl;
            std::cout << "Position error: \n" << cart_error.pos_error << std::endl;
            
            // compute desired velocity in Cartesian space
            Eigen::Matrix<double, 6, 1> x_dot;
            x_dot.head(3) <<  0.5*error.head(3);  
            x_dot.tail(3) <<  0.5*error.tail(3);
            robot_state_.task_references.xdot_g_pos = x_dot;
        }
    };
    void computeActivationFunction(){
        mission_manager_.UpdateMissionData();
        std::vector<std::string> prev_action = mission_manager_.GetPrevAction();
        std::vector<std::string> curr_action = mission_manager_.GetCurrAction();
        double phase_time = mission_manager_.GetPhaseTime();
        // Compute goal vehicle activation matrix
        robot_state_.action_transitions.A_ee=Eigen::Matrix<double, 6, 6>::Identity();
        robot_state_.action_transitions.A_ee *= mission_manager_.ActionTransition("R_M", prev_action, curr_action, phase_time);
    }
    Eigen::Matrix<double,3,1> getGoalDistance(){
        Eigen::Matrix<double, 4,4> w_T_g;
        Eigen::Matrix<double,3,1> goal_distance= Eigen::Matrix<double,3,1>::Zero();
            if(tf_listener_.frameExists("goal_frame")){
                tf::StampedTransform tf;
                Eigen::Affine3d eigen_tf;
                tf_listener_.lookupTransform("panda_R_link0", "goal_frame", ros::Time(0), tf);
                tf::transformTFToEigen(tf, eigen_tf);
                w_T_g =  eigen_tf.matrix();
            
                Eigen::Matrix<double,3,1> goal_distance;
                Eigen::Matrix<double,3,1> current_ee_pose;
                current_ee_pose = robot_state_.trasform_matrices.w_T_e.block<3,1>(0,3);
                goal_distance = w_T_g.block<3,1>(0,3) - current_ee_pose;
                return goal_distance;
            }
            return goal_distance;
    };
    
};
#endif
