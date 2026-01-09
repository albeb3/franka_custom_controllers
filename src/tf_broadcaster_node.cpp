#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

std::string child_frame_id_ = "goal_frame";
std::string frame_id_ = "robot_odom";

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster_node");
  ros::NodeHandle nh;

  tf2_ros::TransformBroadcaster br;

  ros::Rate rate(10.0);
  while (ros::ok()){
    geometry_msgs::TransformStamped transform;

    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = frame_id_;
    transform.child_frame_id = child_frame_id_;

    transform.transform.translation.x = 1.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    br.sendTransform(transform);

    rate.sleep();
  }
  return 0;
}
