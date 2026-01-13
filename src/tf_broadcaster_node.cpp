#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

std::string child_frame_id_left = "goal_frame_panda_L";
std::string frame_id_left = "panda_L_link0";

std::string child_frame_id_right = "goal_frame_panda_R";
std::string frame_id_right = "panda_R_link0";


int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster_node");
  ros::NodeHandle nh;

  tf2_ros::TransformBroadcaster br;

  ros::Rate rate(10.0);
  while (ros::ok()){
    geometry_msgs::TransformStamped transformleft;
    geometry_msgs::TransformStamped transformright;

    transformleft.header.stamp = ros::Time::now();
    transformleft.header.frame_id = frame_id_left;
    transformleft.child_frame_id = child_frame_id_left;
    transformleft.transform.translation.x = 0.7;
    transformleft.transform.translation.y = 0.0;
    transformleft.transform.translation.z = 0.5;
    transformright.header.stamp = ros::Time::now();
    transformright.header.frame_id = frame_id_right;
    transformright.child_frame_id = child_frame_id_right;
    transformright.transform.translation.x = 0.7;
    transformright.transform.translation.y = 0.0;
    transformright.transform.translation.z = 0.5;

    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0);
    transformleft.transform.rotation.x = q.x();
    transformleft.transform.rotation.y = q.y();
    transformleft.transform.rotation.z = q.z();
    transformleft.transform.rotation.w = q.w();
    transformright.transform.rotation.x = q.x();
    transformright.transform.rotation.y = q.y();
    transformright.transform.rotation.z = q.z();
    transformright.transform.rotation.w = q.w();

    br.sendTransform(transformright);
    br.sendTransform(transformleft);

    rate.sleep();
  }
  return 0;
}
