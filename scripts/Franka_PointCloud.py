#!/usr/bin/env python3
import rospy
import open3d as o3d
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import tf.transformations

#fuction to transform point cloud
def transform_point_cloud(parent_frame, child_frame, points):
    try:
        trans, rot = listener.lookupTransform(
            parent_frame, child_frame, rospy.Time(0)
        )
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None

    T = tf.transformations.quaternion_matrix(rot)
    T[:3, 3] = trans

    points = np.asarray(points)
    points_h = np.hstack((points, np.ones((points.shape[0], 1))))
    points_tf = (T @ points_h.T).T[:, :3]

    return points_tf




rospy.init_node('ply_pc_publisher')
listener = tf.TransformListener()
rospy.sleep(1.0)

# Rosparam get robot name
arm_id = rospy.get_param('~arm_id', 'panda_R')


# Carica PLY
ply1 = o3d.io.read_point_cloud('/home/alberto/ros_ws/src/franka_custom_controllers/Franka_pcl/link1.ply')
ply2 = o3d.io.read_point_cloud('/home/alberto/ros_ws/src/franka_custom_controllers/Franka_pcl/link2.ply')
ply3 = o3d.io.read_point_cloud('/home/alberto/ros_ws/src/franka_custom_controllers/Franka_pcl/link3.ply')
ply4 = o3d.io.read_point_cloud('/home/alberto/ros_ws/src/franka_custom_controllers/Franka_pcl/link4.ply')
ply5 = o3d.io.read_point_cloud('/home/alberto/ros_ws/src/franka_custom_controllers/Franka_pcl/link5.ply')
ply6 = o3d.io.read_point_cloud('/home/alberto/ros_ws/src/franka_custom_controllers/Franka_pcl/link6.ply')
ply7 = o3d.io.read_point_cloud('/home/alberto/ros_ws/src/franka_custom_controllers/Franka_pcl/link7.ply')
plyhand = o3d.io.read_point_cloud('/home/alberto/ros_ws/src/franka_custom_controllers/Franka_pcl/hand.ply')
plyfingerright = o3d.io.read_point_cloud('/home/alberto/ros_ws/src/franka_custom_controllers/Franka_pcl/finger.ply')
plyfingerleft = o3d.io.read_point_cloud('/home/alberto/ros_ws/src/franka_custom_controllers/Franka_pcl/finger.ply')

points1 = np.asarray(ply1.points)  # Nx3 array
print("size of points1:", points1.shape)
points2 = np.asarray(ply2.points)  # Nx3 array
print("size of points2:", points2.shape)
points3 = np.asarray(ply3.points)  # Nx3 array
print("size of points3:", points3.shape)
points4 = np.asarray(ply4.points)  # Nx3 array
print("size of points4:", points4.shape)
points5 = np.asarray(ply5.points)  # Nx3 array
print("size of points5:", points5.shape)
points6 = np.asarray(ply6.points)  # Nx3 array
print("size of points6:", points6.shape)
points7 = np.asarray(ply7.points)  # Nx3 array
print("size of points7:", points7.shape)
pointshand = np.asarray(plyhand.points)  # Nx3 array
print("size of pointshand:", pointshand.shape)
pointsfingerright = np.asarray(plyfingerright.points)  # Nx3 array
print("size of pointsfingerright:", pointsfingerright.shape)
pointsfingerleft = np.asarray(plyfingerleft.points)  # Nx3 array
print("size of pointsfingerleft:", pointsfingerleft.shape)
pub = rospy.Publisher(arm_id+'_pc', PointCloud2, queue_size=1)


header0 = std_msgs.msg.Header()
header0.frame_id = arm_id+'_link0'  # il frame del link nel robot

# Crea PointCloud2


# roatation by 90° around x axis
R = ply2.get_rotation_matrix_from_xyz((np.pi/2, 0, 0))
ply1.rotate(R, center=(0, 0, 0))
ply2.rotate(R, center=(0, 0, 0))
ply3.rotate(R, center=(0, 0, 0))
ply4.rotate(R, center=(0, 0, 0))
ply5.rotate(R, center=(0, 0, 0))
ply6.rotate(R, center=(0, 0, 0))
ply7.rotate(R, center=(0, 0, 0))    
plyhand.rotate(R, center=(0, 0, 0))
plyfingerright.rotate(R, center=(0, 0, 0))
plyfingerleft.rotate(R, center=(0, 0, 0))



rate = rospy.Rate(50)
while not rospy.is_shutdown():
    header0.stamp = rospy.Time.now()

    clouds = []

    for frame, ply in [
        (arm_id+'_link1', ply1),
        (arm_id+'_link2', ply2),
        (arm_id+'_link3', ply3),
        (arm_id+'_link4', ply4),
        (arm_id+'_link5', ply5),
        (arm_id+'_link6', ply6),
        (arm_id+'_link7', ply7),
        (arm_id+'_hand', plyhand),
        (arm_id+'_rightfinger', plyfingerright),
        (arm_id+'_leftfinger', plyfingerleft),
    ]:
        pts = transform_point_cloud(
            arm_id+'_link0',
            frame,
            np.asarray(ply.points)
        )

        if pts is None:
            continue

        if pts.ndim == 2 and pts.shape[1] == 3:
            clouds.append(pts)

    if not clouds:
        rate.sleep()
        continue

    points_total = np.vstack(clouds).astype(np.float32)

    pc2_msg = pc2.create_cloud_xyz32(header0, points_total)
    pub.publish(pc2_msg)

    rate.sleep()