#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

std::string child_frame_id_left = "goal_frame_panda_L";
std::string child_generic_frame_id_left = "goal_frame_generic_panda_L";
std::string frame_id_left = "common_link";

std::string child_frame_id_right = "goal_frame_panda_R";
std::string child_generic_frame_id_right = "goal_frame_generic_panda_R";
std::string frame_id_right = "common_link";


int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster_node");
  ros::NodeHandle nh;

  tf2_ros::TransformBroadcaster br;

  ros::Rate rate(10.0);
  while (ros::ok()){
    geometry_msgs::TransformStamped transformleft;
    geometry_msgs::TransformStamped transformright;
    geometry_msgs::TransformStamped transformleft_generic;
    geometry_msgs::TransformStamped transformright_generic;

    transformleft.header.stamp = ros::Time::now();
    transformleft.header.frame_id = "common_link";
    transformleft.child_frame_id = child_frame_id_left;
    transformleft.transform.translation.x = 0.5;
    transformleft.transform.translation.y = 0.2;
    transformleft.transform.translation.z = 0.5;
    transformright.header.stamp = ros::Time::now();
    transformright.header.frame_id = "common_link";
    transformright.child_frame_id = child_frame_id_right;
    transformright.transform.translation.x = 0.5;
    transformright.transform.translation.y = -0.2;
    transformright.transform.translation.z = 0.5;
    transformleft_generic.header.stamp = ros::Time::now();
    transformleft_generic.header.frame_id = "common_link";
    transformleft_generic.child_frame_id = child_generic_frame_id_left;
    transformleft_generic.transform.translation.x = 0.4;
    transformleft_generic.transform.translation.y = 0.2;
    transformleft_generic.transform.translation.z = 0.4;
    transformright_generic.header.stamp = ros::Time::now();
    transformright_generic.header.frame_id = "common_link";
    transformright_generic.child_frame_id = child_generic_frame_id_right;
    transformright_generic.transform.translation.x = 0.5;
    transformright_generic.transform.translation.y = -0.2;
    transformright_generic.transform.translation.z = 0.5;

    tf2::Quaternion q;
    q.setRPY(M_PI, M_PI/4, M_PI/7);
    transformleft.transform.rotation.x = q.x();
    transformleft.transform.rotation.y = q.y();
    transformleft.transform.rotation.z = q.z();
    transformleft.transform.rotation.w = q.w();
    transformright.transform.rotation.x = q.x();
    transformright.transform.rotation.y = q.y();
    transformright.transform.rotation.z = q.z();
    transformright.transform.rotation.w = q.w();
    transformleft_generic.transform.rotation.x = q.x();
    transformleft_generic.transform.rotation.y = q.y();
    transformleft_generic.transform.rotation.z = q.z();
    transformleft_generic.transform.rotation.w = q.w();
    transformright_generic.transform.rotation.x = q.x();
    transformright_generic.transform.rotation.y = q.y();
    transformright_generic.transform.rotation.z = q.z();
    transformright_generic.transform.rotation.w = q.w();

    br.sendTransform(transformright);
    br.sendTransform(transformleft);
    br.sendTransform(transformright_generic);
    br.sendTransform(transformleft_generic);

    rate.sleep();
  }
  return 0;
}
