/*  This script controls the position of the end-effector based on velocity commands received from a Meta joystick in Unity.
    It listens to target angular and translational velocities, integrates them over time, and updates the end-effector pose accordingly.
    The updated pose is broadcasted as a TF transform and published as a PoseStamped message for use by other nodes, such as an interactive joystick controller.
    TOPICS:
    Subscribed:
    - /target_translational_velocity/left_hand (geometry_msgs/Vector3): Receives target translational velocities for the left hand control frame.
    - /target_angular_velocity/left_hand (geometry_msgs/Vector3): Receives target angular velocities for the left hand control frame.
    Published:
    - /joystick_equilibrium_pose (geometry_msgs/PoseStamped): Publishes the updated equilibrium pose of the control frame for use by the interactive joystick controller.
    PARAMETERS:
    - child_frame_id (string, default: "left_hand"): Name of the joystick control frame.
    - frame_id (string, default: "panda_link0"): Name of the robot base frame.
    - deltat (double, default: 0.016): Time step for integrating velocities.
    - limit (double, default: 0.6): Maximum allowed velocity for saturation.
*/

#include "tf/transform_broadcaster.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include <iostream>
#include <geometry_msgs/Transform.h>
#include <std_msgs/Bool.h>
#include <string>
#include <boost/bind/bind.hpp>
#include <tf/transform_listener.h>

// CONFIGURATION PARAMETERS-------------------------------------------------------------------------------------------------------
// child_frame_id_: name of the joystick control frame (e.g., left_hand or right_hand)
// frame_id_: name of the robot base frame (e.g., panda_link0)
std::string child_frame_id_ = "right_hand";
std::string frame_id_ = "panda_R_link0";
// time step for integrating velocities
double deltat_ = 0.016;
// maximum allowed velocity for saturation
double limit_ = 0.6;

// Variables for storing the current pose of the control frame
geometry_msgs::TransformStamped transform_hand_displayed_; // transform to be broadcasted
geometry_msgs::PoseStamped transform_joystick; // pose to be published to the interactive_joystick node

// DECLARATION OF GLOBAL VARIABLES 
// variables for storing the current position and orientation of the control frame
double translation_x_ ;
double translation_y_ ;
double translation_z_ ;
double rotation_x_ ;
double rotation_y_ ;
double rotation_z_ ;
double rotation_w_;
//--------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS----------------------------------------------------------------------------------------------------------------------
// Function to set the initial configuration of the control frame
void StartConfiguration(geometry_msgs::TransformStamped start_PandaLink0_T_PandaEE){
    translation_x_ = start_PandaLink0_T_PandaEE.transform.translation.x;
    translation_y_ = start_PandaLink0_T_PandaEE.transform.translation.y;
    translation_z_ = start_PandaLink0_T_PandaEE.transform.translation.z;
    rotation_x_ = start_PandaLink0_T_PandaEE.transform.rotation.x;
    rotation_y_ = start_PandaLink0_T_PandaEE.transform.rotation.y;
    rotation_z_ = start_PandaLink0_T_PandaEE.transform.rotation.z;
    rotation_w_ = start_PandaLink0_T_PandaEE.transform.rotation.w;
}
void SetConfiguration(tf::TransformListener& tf_listener){
    tf::StampedTransform start_PandaLink0_T_PandaEE;
    try {
        tf_listener.lookupTransform("panda_R_link0", "panda_R_EE", ros::Time(0), start_PandaLink0_T_PandaEE);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("TF lookup failed: %s", ex.what());
        return;
    }
    translation_x_ = start_PandaLink0_T_PandaEE.getOrigin().x();
    translation_y_ = start_PandaLink0_T_PandaEE.getOrigin().y();
    translation_z_ = start_PandaLink0_T_PandaEE.getOrigin().z();
    rotation_x_ = start_PandaLink0_T_PandaEE.getRotation().x();
    rotation_y_ = start_PandaLink0_T_PandaEE.getRotation().y();
    rotation_z_ = start_PandaLink0_T_PandaEE.getRotation().z();
    rotation_w_ = start_PandaLink0_T_PandaEE.getRotation().w();
}
// function to set transform matrix of EE pose wrt base frame
void Callback_return_to_mission(const std_msgs::Bool::ConstPtr& msg, tf::TransformListener* tf_listener){
    if (msg->data){
    std::cout << "Return to mission command received for " << child_frame_id_ << std::endl;

        // Reset the control frame to the initial configuration
        SetConfiguration(*tf_listener);
    }
}
void Callback_follow_EE(const std_msgs::Bool::ConstPtr& msg, tf::TransformListener* tf_listener){
    if(msg->data){
        std::cout << "Received velocity command" << child_frame_id_ << std::endl;
    }
    if (!msg->data){
        std::cout << "Follow EE command received for " << child_frame_id_ << std::endl;

        // Reset the control frame to the current EE configuration
        SetConfiguration(*tf_listener);
    }
}
// Saturation function to limit the maximum velocity
double saturate(double val)
{ 
    if (val > limit_)   return limit_;
    else if (val < -limit_) return -limit_;
    else    return val;
}

// Callback function for receiving joystick translational velocity commands
void Callback_target_translational_velocity(const geometry_msgs::Vector3::ConstPtr& msg)
{
    translation_x_ += (deltat_*saturate(msg->z));
    translation_y_ += (deltat_*saturate(-msg->x));
    translation_z_ += (deltat_*saturate(msg->y));
}

// Callback function for receiving joystick angular velocity commands
void Callback_target_angular_velocity(const geometry_msgs::Vector3::ConstPtr& msg){
    // setting current orientation as a tf2 quaternion
    tf2::Quaternion q_current(  rotation_x_, rotation_y_, rotation_z_, rotation_w_ ); 
    // Calculate angle increments
    double delta_roll = deltat_ * saturate(msg->z);
    double delta_pitch = -deltat_ * saturate(msg->x);
    double delta_yaw =  deltat_ * saturate(msg->y);
    // Create a quaternion for the incremental rotation
    tf2::Quaternion q_delta;
    // Transform incremental angles to quaternion
    q_delta.setRPY(delta_roll, delta_pitch, delta_yaw);
    // Apply the incremental rotation to the current orientation
    tf2::Quaternion q_new = q_delta * q_current;
    // Normalize the resulting quaternion
    q_new.normalize();
    // Update global orientation variables
    rotation_x_ = q_new.x();
    rotation_y_ = q_new.y();
    rotation_z_ = q_new.z();
    rotation_w_ = q_new.w();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_EE_velocity_node_Right");
    ros::NodeHandle node;
    node.param("child_frame_id", child_frame_id_, std::string("right_hand"));
    node.param("frame_id", frame_id_, std::string("panda_R_link0"));
    node.param("deltat", deltat_, 0.016);
    node.param("limit", limit_, 0.6);
    
    // Definition of TF listener and broadcaster for getting the initial pose and publishing the updated one 
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster broadcaster;
    std::shared_ptr<tf::TransformListener> tf_listener_ptr= std::make_shared<tf::TransformListener>();
  
    // Routine for setting the initial pose of the control frame as the current pose of the end-effector
    // -------------------------------------------------------->USE StartConfiguration(start_PandaLink0_T_PandaEE) TO RESET THE INITIAL POSE!!!!
    ROS_INFO("Waiting for panda_R_link0_T_panda_R_EE...");
    tfBuffer.canTransform("panda_R_link0", "panda_R_EE", ros::Time(0), ros::Duration(10.0));
    geometry_msgs::TransformStamped start_PandaLink0_T_PandaEE;
    try {
        start_PandaLink0_T_PandaEE = tfBuffer.lookupTransform("panda_R_link0", "panda_R_EE", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_ERROR("TF lookup failed: %s", ex.what());
        ros::shutdown();
        return 0;
    }
    StartConfiguration(start_PandaLink0_T_PandaEE);

    // ROS Subscribers and Publishers definition
    // pub_PositionJoystick publishes the equilibrium pose to the interactive_joystick node of franka_example_controllers package (my customized version of the original interactive_marker node)
    //ros::Publisher pub_PositionJoystick = node.advertise<geometry_msgs::PoseStamped>("panda_R/joystick_equilibrium_pose", 10);
    // Subscribers for receiving target angular and translational velocities of meta joystick commands from Unity
    ros::Subscriber sub_angular_velocity = node.subscribe("/target_angular_velocity/"+child_frame_id_, 10, Callback_target_angular_velocity);
    ros::Subscriber sub_translational_velocity = node.subscribe("/target_translational_velocity/"+child_frame_id_, 10, Callback_target_translational_velocity);
    ros::Subscriber sub_return_to_mission = node.subscribe<std_msgs::Bool>("/panda_R/my_joint_velocity_controller/return_to_mission/", 10,boost::bind(&Callback_return_to_mission, _1, tf_listener_ptr.get()));
    ros::Subscriber sub_follow_EE = node.subscribe<std_msgs::Bool>("/panda_R/my_joint_velocity_controller/teleop_cmd_received/", 10,boost::bind(&Callback_follow_EE, _1, tf_listener_ptr.get()));

    ros::Rate rate(100.0);

    while (ros::ok()){// prepare a single TransformStamped and send it
        // Update and publish the transform and pose based on the integrated velocities
        transform_hand_displayed_.header.frame_id = frame_id_;
        transform_hand_displayed_.child_frame_id = "panda_R_teleop_frame";
        transform_hand_displayed_.header.stamp = ros::Time::now();
        transform_hand_displayed_.transform.translation.x = translation_x_;
        transform_hand_displayed_.transform.translation.y = translation_y_;
        transform_hand_displayed_.transform.translation.z = translation_z_;
        geometry_msgs::Quaternion quat;
        quat.x = rotation_x_;
        quat.y = rotation_y_;
        quat.z = rotation_z_;
        quat.w = rotation_w_;
        transform_hand_displayed_.transform.rotation = quat;
        // Prepare the PoseStamped message for the interactive joystick controller
        transform_joystick.header.frame_id = frame_id_;
        transform_joystick.header.stamp = ros::Time::now();
        transform_joystick.pose.position.x = translation_x_;
        transform_joystick.pose.position.y = translation_y_;
        transform_joystick.pose.position.z = translation_z_;
        transform_joystick.pose.orientation = quat;
        // Broadcast the updated transform and publish the pose
        broadcaster.sendTransform(transform_hand_displayed_);
        //pub_PositionJoystick.publish(transform_joystick);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

  