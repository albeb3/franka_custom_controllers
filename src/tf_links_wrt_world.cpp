#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_tf_wrt_world_node");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Publisher tf_pub =
      nh.advertise<tf2_msgs::TFMessage>("/panda_tf_wrt_world", 10);

  ros::Rate rate(100.0);

  ROS_INFO("Publishing panda_link TFs wrt world on /panda_tf_wrt_world");

  while (ros::ok())
  {
    tf2_msgs::TFMessage tf_msg;
    tf_msg.transforms.clear();

    for (int i = 0; i <= 7; i++)
    {
      std::string link_name = "panda_link" + std::to_string(i);

      try
      {
        geometry_msgs::TransformStamped tf_world_link =
            tfBuffer.lookupTransform("world", link_name, ros::Time(0));

        tf_msg.transforms.push_back(tf_world_link);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN_THROTTLE(1.0,
          "TF lookup failed for %s: %s", link_name.c_str(), ex.what());
      }
    }

    if (!tf_msg.transforms.empty())
      tf_pub.publish(tf_msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
