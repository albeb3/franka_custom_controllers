#include "tf/transform_broadcaster.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_broadcaster.h"
#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include <iostream>
#include <geometry_msgs/Transform.h>
#include <string>

std::string child_frame_id_ = "right_hand";
std::string frame_id_ = "panda_link0";

double translation_x_ = 0.4;
double translation_y_ = 0.0;
double translation_z_ = 0.5;
double rotation_x_ = 1.0;
double rotation_y_ = 0.0;
double rotation_z_ = 0.0;
double rotation_w = 0.0;

// Variables for storing the current pose of the control frame
geometry_msgs::TransformStamped transform_hand_displayed_; // transform to be broadcasted
geometry_msgs::PoseStamped transform_joystick; // pose to be published to the interactive_joystick node

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_EE_node");
  ros::NodeHandle node;
 

  
  tf2_ros::TransformBroadcaster broadcaster;
  std::cout << "Starting tf broadcaster from " << frame_id_ << " to " << child_frame_id_ << std::endl;
  ros::Publisher pub_PositionJoystick = node.advertise<geometry_msgs::PoseStamped>("joystick_equilibrium_pose", 10);

  ros::Rate rate(10.0);
  while (ros::ok()){// prepare a single TransformStamped and send it
    
    transform_hand_displayed_.header.frame_id = frame_id_;
    transform_hand_displayed_.child_frame_id = child_frame_id_;
    transform_hand_displayed_.transform.translation.x = translation_x_;
    transform_hand_displayed_.transform.translation.y = translation_y_;
    transform_hand_displayed_.transform.translation.z = translation_z_;
    geometry_msgs::Quaternion quat;
    quat.x = rotation_x_;
    quat.y = rotation_y_;
    quat.z = rotation_z_;
    quat.w = rotation_w;
    transform_hand_displayed_.transform.rotation = quat;
    transform_hand_displayed_.header.stamp = ros::Time::now();
    // Prepare the PoseStamped message for the interactive joystick controller
        transform_joystick.header.frame_id = frame_id_;
        transform_joystick.header.stamp = ros::Time::now();
        transform_joystick.pose.position.x = translation_x_;
        transform_joystick.pose.position.y = translation_y_;
        transform_joystick.pose.position.z = translation_z_;
        transform_joystick.pose.orientation = quat;

  broadcaster.sendTransform(transform_hand_displayed_);
  pub_PositionJoystick.publish(transform_joystick);
  
  ros::spinOnce();
  rate.sleep();
}
  return 0;
}

  