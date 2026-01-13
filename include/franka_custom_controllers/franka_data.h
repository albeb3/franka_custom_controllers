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
    Eigen::Matrix<double, 4,4> b_T_e,b_T_g,b_T_gteleop;
};

struct Jacobians{
    // Jacobian of goal position projected on vehicle frame
    Eigen::Matrix<double, 6, 7> Jee ;
    Eigen::Matrix<double, 7, 7> Jjl;
    Eigen::Matrix<double, 6, 7> Jteleop;
};
struct Action_transitions{
    Eigen::Matrix<double, 6,6> A_ee;
    Eigen::Matrix<double, 7,7> A_jl;
    Eigen::Matrix<double, 6,6> A_teleop;
};
struct Task_references{
    Eigen::Matrix<double, 6,1> xdot_g_pos;
    Eigen::Matrix<double, 7,1> xdot_jl;
    Eigen::Matrix<double, 6,1> xdot_teleop;
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
    
        tf::TransformListener& tf_listener_;
        ros::NodeHandle& nh_;
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        MissionManager& mission_manager_;
        robot_state robot_state_;
        std::string arm_id_;
        // teleop variables
        bool teleop_cmd_received_= false;
        bool return_to_mission_= false;
        ros::Subscriber teleop_cmd_sub_;
        ros::Subscriber return_to_mission_sub_;
        // teleop callbacks
        void teleopCmdCallback(const std_msgs::Bool::ConstPtr& msg){
            //std::cout << "Teleop command received: " << msg->data << std::endl;
            teleop_cmd_received_ = msg->data;
        };
        void returnToMissionCallback(const std_msgs::Bool::ConstPtr& msg){
            
            return_to_mission_ = msg->data;
            std::cout << "Return to mission command received: " << return_to_mission_ << std::endl;
        };
        

        
        void computeJacobian(){
            // get jacobian 
            std::array<double, 42> jacobian_array =
                model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
            Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
            robot_state_.jacobians.Jee = jacobian;
            // set joint limit jacobian as identity
            robot_state_.jacobians.Jjl = Eigen::Matrix<double,7,7>::Identity(); 
            // set teleop jacobian as end-effector jacobian
            robot_state_.jacobians.Jteleop = jacobian;
        };
        void updateTransform(){
            // update end-effector transform
            franka::RobotState robot_state = state_handle_->getRobotState();
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            robot_state_.trasform_matrices.b_T_e = transform.matrix();
            // update goal transform if exists
            if(GetGoalFrameExists("goal_frame")){
                tf::StampedTransform tf;
                Eigen::Affine3d eigen_tf;
                try{
                    tf_listener_.lookupTransform(arm_id_+"_link0", "goal_frame_" + arm_id_, ros::Time(0), tf);
                    tf::transformTFToEigen(tf, eigen_tf);
                    robot_state_.trasform_matrices.b_T_g =  eigen_tf.matrix();
                    
                }
                catch (tf::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    return;
                }
            }
            if(GetGoalFrameExists(arm_id_+"_teleop_frame")){
                tf::StampedTransform tf;
                Eigen::Affine3d eigen_tf;
                try{
                    tf_listener_.lookupTransform(arm_id_+"_link0", arm_id_+"_teleop_frame", ros::Time(0), tf);
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
            // compute goal position velocity if goal frame exists
            switch (mission_manager_.GetPhase())
            {
            case 0: // approach phase
                {
                robot_state_.task_references.xdot_g_pos = Eigen::Matrix<double,6,1>::Zero();
                break; 
                }
            case 1: // reaching goal phase
                {
                CartErrorResult cart_error = CartError( robot_state_.trasform_matrices.b_T_g,robot_state_.trasform_matrices.b_T_e);
                Eigen::Matrix<double, 6, 1> error ;
                error.head(3) = cart_error.pos_error;
                error.tail(3) = cart_error.ori_error;
                Eigen::Matrix<double, 6, 1> x_dot;
                x_dot.head(3) <<  0.9*error.head(3);  
                x_dot.tail(3) <<  0.9*error.tail(3);
                x_dot = Saturate( x_dot, 0.8);
                robot_state_.task_references.xdot_g_pos = x_dot;
                }
                break;
            case 2: // stop phase
                {
                robot_state_.task_references.xdot_g_pos = Eigen::Matrix<double,6,1>::Zero();
                break; 
                }
            case 3: // teleop phase
                {
                CartErrorResult cart_error_teleop = CartError( robot_state_.trasform_matrices.b_T_gteleop,robot_state_.trasform_matrices.b_T_e);
                Eigen::Matrix<double, 6, 1> error_teleop ;
                error_teleop.head(3) = cart_error_teleop.pos_error;
                error_teleop.tail(3) = cart_error_teleop.ori_error;
                Eigen::Matrix<double, 6, 1> x_dot_teleop;
                x_dot_teleop.head(3) <<  5*error_teleop.head(3);  
                x_dot_teleop.tail(3) <<  5*error_teleop.tail(3);
                x_dot_teleop = Saturate( x_dot_teleop, 0.8);
                robot_state_.task_references.xdot_teleop = x_dot_teleop;
                }
                break;
            default:
                break;
            }
        };
        void computeActivationFunction(){
        mission_manager_.UpdateMissionData();
        std::vector<std::string> prev_action = mission_manager_.GetPrevAction();
        std::vector<std::string> curr_action = mission_manager_.GetCurrAction();
        double phase_time = mission_manager_.GetPhaseTime();
        // Compute joint limit activation matrix
        robot_state_.action_transitions.A_jl=Eigen::Matrix<double, 7, 7>::Identity();
        for (size_t i=0; i<7;i++){
            robot_state_.action_transitions.A_jl(i,i)=IncreasingBellShapedFunction(0.9*robot_state_.joint_upper_limits(i), robot_state_.joint_upper_limits(i),
                                                                                 0.0, 1.0, robot_state_.q(i)) +
                                                    DecreasingBellShapedFunction(robot_state_.joint_lower_limits(i), 0.9*robot_state_.joint_lower_limits(i),
                                                                                 0.0, 1.0, robot_state_.q(i));
        }
        robot_state_.action_transitions.A_jl *= mission_manager_.ActionTransition("J_L", prev_action, curr_action, phase_time);
        
        // Compute goal vehicle activation matrix
        robot_state_.action_transitions.A_ee=Eigen::Matrix<double, 6, 6>::Identity();
        robot_state_.action_transitions.A_ee *= mission_manager_.ActionTransition("R_M", prev_action, curr_action, phase_time);
        // Compute teleop activation matrix
        robot_state_.action_transitions.A_teleop=Eigen::Matrix<double, 6, 6>::Identity();
        robot_state_.action_transitions.A_teleop *= mission_manager_.ActionTransition("T_M", prev_action, curr_action, phase_time);
    }
    
    public:
        franka_state(ros::NodeHandle& nh, MissionManager& mission_manager, tf::TransformListener& tf_listener): nh_(nh), mission_manager_(mission_manager), tf_listener_(tf_listener){
            teleop_cmd_sub_ = nh_.subscribe<std_msgs::Bool>("teleop_cmd_received", 1, &franka_state::teleopCmdCallback, this);
            return_to_mission_sub_ = nh_.subscribe<std_msgs::Bool>("return_to_mission", 1, &franka_state::returnToMissionCallback, this);
        };
        ~franka_state()= default;
        bool init(hardware_interface::RobotHW* robot_hardware, std::string arm_id){
            arm_id_ = arm_id;
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
                    "CartesianImpedanceExampleController: Exception getting state handle from interface: "
                    << ex.what());
                return false;
            }
            return true;

        };
        void starting(){
            // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
            // to initial configuration
            franka::RobotState initial_state = state_handle_->getRobotState();
            // get jacobian
            std::array<double, 42> jacobian_array =
                model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
            // convert to eigen
            Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
            Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
            Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
            
            
        }
        void update(){
            updateTransform();
            computeJacobian();
            if(GetGoalFrameExists("goal_frame")){
                computeTaskReference();
                computeActivationFunction();
                }
            if(GetGoalFrameExists(arm_id_+"_teleop_frame")  && GetTeleopCmdReceived()){
                std::cout << "Teleop control active for: " << arm_id_ <<"_teleop_frame" <<std::endl;
                computeTaskReference();
                computeActivationFunction();
                }
            updateState();
            
        };
        void updateState(){
            for (size_t i=0; i<7; i++){
                robot_state_.q(i) = state_handle_->getRobotState().q[i];
            }
        };
        bool GetGoalFrameExists(const std::string& goal){
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
                CartErrorResult cart_error = CartError( robot_state_.trasform_matrices.b_T_g,robot_state_.trasform_matrices.b_T_e);
                Eigen::Matrix<double,6,1> goal_distance;
                goal_distance.head<3>() = cart_error.pos_error;
                goal_distance.tail<3>() = cart_error.ori_error;
                return goal_distance.squaredNorm();
                
                }
            else if (goal == "teleop"){
                CartErrorResult cart_error = CartError( robot_state_.trasform_matrices.b_T_gteleop,robot_state_.trasform_matrices.b_T_e);
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
        Eigen::MatrixXd getJacobian(const std::string task_name){
            if (task_name == "goal_move"){
                return robot_state_.jacobians.Jee;
            }
            else if (task_name == "joint_limit"){
                return robot_state_.jacobians.Jjl;
            }
            else if (task_name == "teleop"){
                return robot_state_.jacobians.Jteleop;
            }
            else{
                return Eigen::MatrixXd::Zero(6,7);
            }
        };
        Eigen::MatrixXd getActivationMatrix(const std::string task_name){
            if (task_name == "goal_move"){
                return robot_state_.action_transitions.A_ee;
            }
            else if (task_name == "joint_limit"){
                return robot_state_.action_transitions.A_jl;
            }
            else if (task_name == "teleop"){
                return robot_state_.action_transitions.A_teleop;
            }
            else{
                return Eigen::MatrixXd::Zero(6,6);
            }
        };
        Eigen::MatrixXd getTaskReference(const std::string task_name){
            if (task_name == "goal_move"){
                return robot_state_.task_references.xdot_g_pos;
            }
            else if (task_name == "joint_limit"){
                return robot_state_.task_references.xdot_jl;
            }
            else if (task_name == "teleop"){
                return robot_state_.task_references.xdot_teleop;
            }
            else{
                return Eigen::MatrixXd::Zero(6,1);
            }
        };
        void SetTaskReferncesToZero(){
            robot_state_.task_references.xdot_g_pos = Eigen::Matrix<double,6,1>::Zero();
            robot_state_.task_references.xdot_jl = Eigen::Matrix<double,7,1>::Zero();
            robot_state_.task_references.xdot_teleop = Eigen::Matrix<double,6,1>::Zero();
        };
        bool GetReturnToMission(){
            return return_to_mission_;
        };
        bool GetTeleopCmdReceived(){
            return teleop_cmd_received_;
        };
        void SetReturnToMission(bool val){
            return_to_mission_ = val;
        };
        void SetTeleopCmdReceived(bool val){
            teleop_cmd_received_ = val;
        };
        
};
#endif
