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
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include <iostream>
#include <geometry_msgs/Transform.h>
#include <string>
#include <std_msgs/Bool.h>
#include <boost/bind/bind.hpp>

// CONFIGURATION PARAMETERS-------------------------------------------------------------------------------------------------------
// frame_id_: name of the robot base frame (e.g., panda_link0)

std::string arm_id_;
// time step for integrating velocities
//double deltat_ = 0.016;
double deltat_ = 0.016; // to be consistent with franka control loop of 1kHz
// maximum allowed velocity for saturation
double limit_ = 0.6;

// Variables for storing the current pose of the control frame
geometry_msgs::TransformStamped transform_hand_displayed_; // transform to be broadcasted


// DECLARATION OF GLOBAL VARIABLES 
// variables for storing the current position and orientation of the control frame
double translation_x_= 0.0 ;
double translation_y_ = 0.0;
double translation_z_ = 0.0;
double rotation_x_ = 0.0;
double rotation_y_ =0.0;
double rotation_z_ = 0.0;
double rotation_w_=1.0;

 
 bool alignedTeleopFrame = true;
//--------------------------------------------------------------------------------------------------------------------------------
// FUNCTIONS----------------------------------------------------------------------------------------------------------------------
// Function to set the initial configuration of the control frame
void SetConfiguration(geometry_msgs::TransformStamped start_PandaLink0_T_PandaEE){
    translation_x_ = start_PandaLink0_T_PandaEE.transform.translation.x;
    translation_y_ = start_PandaLink0_T_PandaEE.transform.translation.y;
    translation_z_ = start_PandaLink0_T_PandaEE.transform.translation.z;
    rotation_x_ = start_PandaLink0_T_PandaEE.transform.rotation.x;
    rotation_y_ = start_PandaLink0_T_PandaEE.transform.rotation.y;
    rotation_z_ = start_PandaLink0_T_PandaEE.transform.rotation.z;
    rotation_w_ = start_PandaLink0_T_PandaEE.transform.rotation.w;
}

// function to set transform matrix of EE pose wrt base frame




void allignTeleopFrameWithEE(tf2_ros::Buffer& tfBuffer){
    //if(tfBuffer.canTransform(arm_id_ + "_link0", arm_id_ + "_EE_teleop", ros::Time(0))){
      if(tfBuffer.canTransform("world", arm_id_ + "_EE_teleop", ros::Time(0))){
        geometry_msgs::TransformStamped start_PandaLink0_T_PandaEE;
        try {
            start_PandaLink0_T_PandaEE = tfBuffer.lookupTransform("world", arm_id_ + "_EE_teleop", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_ERROR("tf_Broadcaster: TF lookup failed: %s", ex.what());
            ros::shutdown();
            return;
        }
    SetConfiguration(start_PandaLink0_T_PandaEE);
    return;
    }
}
void setParameters(ros::NodeHandle& private_node){
    if(!private_node.getParam("arm_id", arm_id_)){
        ROS_ERROR("Parameter arm_id not set");
        return ;
    }

    if(!private_node.getParam("deltat", deltat_)){
        ROS_ERROR("Parameter deltat not set");
        return ;
    }
    if(!private_node.getParam("limit", limit_)){
        ROS_ERROR("Parameter limit not set");
        return ;
    }
    private_node.param("deltat", deltat_, 0.016);
    private_node.param("limit", limit_, 0.6);
    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_EE_velocity_node");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    std::string arm_id;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    //setParameters(private_node);
    arm_id_= "panda_R"; // to be consistent with franka_example_controllers
    // Definition of TF listener and broadcaster for getting the initial pose and publishing the updated one 
   
    tf2_ros::TransformBroadcaster broadcaster;
    std::shared_ptr<tf::TransformListener> tf_listener_ptr= std::make_shared<tf::TransformListener>();

    
    // Routine for setting the initial pose of the control frame as the current pose of the end-effector
    // -------------------------------------------------------->USE StartConfiguration(start_PandaLink0_T_PandaEE) TO RESET THE INITIAL POSE!!!!
    //ROS_INFO("Waiting for %s_link0_T_%s_EE...", arm_id_.c_str(), arm_id_.c_str());
    
    // ROS Subscribers and Publishers definition
    // pub_PositionJoystick publishes the equilibrium pose to the interactive_joystick node of franka_example_controllers package (my customized version of the original interactive_marker node)
    // Subscribers for receiving target angular and translational velocities of meta joystick commands from Unity
   ros::Rate rate(100.0);

    while (ros::ok()){// prepare a single TransformStamped and send it
        
        allignTeleopFrameWithEE(tfBuffer);
            
        
        
         // Update the pose of the control frame by integrating the received velocity commands over time
         // Integration is performed in the callback functions
        
         // Prepare the TransformStamped message to be broadcasted and pubished for joystick equilibri
        
        // Update and publish the transform and pose based on the integrated velocities
        //transform_hand_displayed_.header.frame_id = arm_id_ + "_link0";
        transform_hand_displayed_.child_frame_id = arm_id_ + "_teleop_frame";
        transform_hand_displayed_.header.frame_id = "world";
        transform_hand_displayed_.header.stamp = ros::Time::now();
        transform_hand_displayed_.transform.translation.x = translation_x_;
        transform_hand_displayed_.transform.translation.y = translation_y_;
        transform_hand_displayed_.transform.translation.z = translation_z_;
        tf2::Quaternion quat;
        quat.setX(rotation_x_);
        quat.setY(rotation_y_);
        quat.setZ(rotation_z_);
        quat.setW(rotation_w_);
        //rotate around x axis by 180 deg to have the teleop frame aligned with the EE frame
        tf2::Quaternion rot_180_x;
        rot_180_x.setRPY(M_PI, 0.0, 0.0);
        quat = rot_180_x * quat;
        quat.normalize();
        geometry_msgs::Quaternion quat_msg;
        quat_msg.x = quat.x();
        quat_msg.y = quat.y();
        quat_msg.z = quat.z();
        quat_msg.w = quat.w();
        transform_hand_displayed_.transform.rotation = quat_msg;
        // Broadcast the updated transform and publish the pose
        broadcaster.sendTransform(transform_hand_displayed_);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

  