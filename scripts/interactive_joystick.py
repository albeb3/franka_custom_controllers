#!/usr/bin/env python3

import rospy
import tf.transformations
import tf2_ros
import numpy as np

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped, TransformStamped
from franka_msgs.msg import FrankaState

transformStamped = TransformStamped()
marker_pose = PoseStamped()
pose_pub = None
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]


def publisher_callback(msg, link_name):
    marker_pose.header.frame_id = link_name
    marker_pose.header.stamp = rospy.Time.now()
    pose_pub.publish(marker_pose)

def publisher_callback_TF(msg, link_name, broadcaster):
    transformStamped.header.frame_id = link_name+"_link0"
    transformStamped.child_frame_id = link_name+"_joystick_EE"
    transformStamped.header.stamp = rospy.Time.now()
    transformStamped.transform.translation.x = marker_pose.pose.position.x
    transformStamped.transform.translation.y = marker_pose.pose.position.y
    transformStamped.transform.translation.z = marker_pose.pose.position.z
    transformStamped.transform.rotation = marker_pose.pose.orientation
    broadcaster.sendTransform(transformStamped)
def joystick_callback(msg):
    marker_pose.pose = msg.pose
  

def wait_for_initial_pose():
    msg = rospy.wait_for_message("franka_state_controller/franka_states",
                                 FrankaState)  # type: FrankaState

    initial_quaternion = \
        tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    initial_quaternion = initial_quaternion / \
        np.linalg.norm(initial_quaternion)
    marker_pose.pose.orientation.x = initial_quaternion[0]
    marker_pose.pose.orientation.y = initial_quaternion[1]
    marker_pose.pose.orientation.z = initial_quaternion[2]
    marker_pose.pose.orientation.w = initial_quaternion[3]
    marker_pose.pose.position.x = msg.O_T_EE[12]
    marker_pose.pose.position.y = msg.O_T_EE[13]
    marker_pose.pose.position.z = msg.O_T_EE[14]


if __name__ == "__main__":
    rospy.init_node("equilibrium_pose_node")
    listener = tf.TransformListener()
    #link_name = "joystick_link_name"
    link_name = rospy.get_param("~link_name", "panda_L")

    wait_for_initial_pose()

    pose_pub = rospy.Publisher(
        "equilibrium_pose", PoseStamped, queue_size=10)
    rospy.Subscriber(
        "joystick_equilibrium_pose", PoseStamped, joystick_callback,
        queue_size=10)
    broadcaster = tf2_ros.TransformBroadcaster()
   
    #broadcaster.sendTransform(transformStamped)


    # run pose publisher
    rospy.Timer(rospy.Duration(0.005),
                lambda msg: publisher_callback(msg, link_name))
    #run TF broadcaster
    rospy.Timer(rospy.Duration(0.02),
                lambda msg: publisher_callback_TF(msg, link_name,broadcaster))
  
    rospy.spin()
