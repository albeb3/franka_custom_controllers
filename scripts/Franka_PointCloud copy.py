#!/usr/bin/env python3
import rospy
import open3d as o3d
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import std_msgs.msg




rospy.init_node('ply_pc_publisher')

# Rosparam get robot name
arm_id = rospy.get_param('~arm_id', 'panda_R')


# Carica PLY
ply0 = o3d.io.read_point_cloud('/home/alberto/prova_ws/src/franka_custom_controllers/Franka_pcl/link0.ply')
ply1 = o3d.io.read_point_cloud('/home/alberto/prova_ws/src/franka_custom_controllers/Franka_pcl/link1.ply')
ply2 = o3d.io.read_point_cloud('/home/alberto/prova_ws/src/franka_custom_controllers/Franka_pcl/link2.ply')
ply3 = o3d.io.read_point_cloud('/home/alberto/prova_ws/src/franka_custom_controllers/Franka_pcl/link3.ply')
ply4 = o3d.io.read_point_cloud('/home/alberto/prova_ws/src/franka_custom_controllers/Franka_pcl/link4.ply')
ply5 = o3d.io.read_point_cloud('/home/alberto/prova_ws/src/franka_custom_controllers/Franka_pcl/link5.ply')
ply6 = o3d.io.read_point_cloud('/home/alberto/prova_ws/src/franka_custom_controllers/Franka_pcl/link6.ply')
ply7 = o3d.io.read_point_cloud('/home/alberto/prova_ws/src/franka_custom_controllers/Franka_pcl/link7.ply')
plyhand = o3d.io.read_point_cloud('/home/alberto/prova_ws/src/franka_custom_controllers/Franka_pcl/hand.ply')
plyfingerright = o3d.io.read_point_cloud('/home/alberto/prova_ws/src/franka_custom_controllers/Franka_pcl/finger.ply')
plyfingerleft = o3d.io.read_point_cloud('/home/alberto/prova_ws/src/franka_custom_controllers/Franka_pcl/finger.ply')
points0 = np.asarray(ply0.points)  # Nx3 array
points1 = np.asarray(ply1.points)  # Nx3 array
points2 = np.asarray(ply2.points)  # Nx3 array
points3 = np.asarray(ply3.points)  # Nx3 array
points4 = np.asarray(ply4.points)  # Nx3 array
points5 = np.asarray(ply5.points)  # Nx3 array
points6 = np.asarray(ply6.points)  # Nx3 array
points7 = np.asarray(ply7.points)  # Nx3 array
pointshand = np.asarray(plyhand.points)  # Nx3 array
pointsfingerright = np.asarray(plyfingerright.points)  # Nx3 array
pointsfingerleft = np.asarray(plyfingerleft.points)  # Nx3 array
pub = rospy.Publisher('link0_pc', PointCloud2, queue_size=1)
pub1 = rospy.Publisher('link1_pc', PointCloud2, queue_size=1)
pub2 = rospy.Publisher('link2_pc', PointCloud2, queue_size=1)
pub3 = rospy.Publisher('link3_pc', PointCloud2, queue_size=1)
pub4 = rospy.Publisher('link4_pc', PointCloud2, queue_size=1)
pub5 = rospy.Publisher('link5_pc', PointCloud2, queue_size=1)
pub6 = rospy.Publisher('link6_pc', PointCloud2, queue_size=1)
pub7 = rospy.Publisher('link7_pc', PointCloud2, queue_size=1)
pubhand = rospy.Publisher('hand_pc', PointCloud2, queue_size=1)
pubfingerright = rospy.Publisher('fingerright_pc', PointCloud2, queue_size=1)
pubfingerleft = rospy.Publisher('fingerleft_pc', PointCloud2, queue_size=1)
header0 = std_msgs.msg.Header()
header0.frame_id = arm_id+'_link0'  # il frame del link nel robot
header1 = std_msgs.msg.Header()
header1.frame_id = arm_id+'_link1'
header2 = std_msgs.msg.Header()
header2.frame_id = arm_id+'_link2'
header3 = std_msgs.msg.Header()
header3.frame_id = arm_id+'_link3'
header4 = std_msgs.msg.Header()
header4.frame_id = arm_id+'_link4'
header5 = std_msgs.msg.Header()
header5.frame_id = arm_id+'_link5'
header6 = std_msgs.msg.Header()
header6.frame_id = arm_id+'_link6'
header7 = std_msgs.msg.Header()
header7.frame_id = arm_id+'_link7'
headerhand = std_msgs.msg.Header()
headerhand.frame_id = arm_id+'_hand'
headerfinger1 = std_msgs.msg.Header()
headerfinger1.frame_id = arm_id+'_rightfinger'
headerfinger2 = std_msgs.msg.Header()
headerfinger2.frame_id = arm_id+'_leftfinger'

# Crea PointCloud2
pc2_msg = pc2.create_cloud_xyz32(header0, points0)
pc2_msg1 = pc2.create_cloud_xyz32(header1, points1)
pc2_msg2 = pc2.create_cloud_xyz32(header2, points2)
pc2_msg3 = pc2.create_cloud_xyz32(header3, points3)
pc2_msg4 = pc2.create_cloud_xyz32(header4, points4)
pc2_msg5 = pc2.create_cloud_xyz32(header5, points5)
pc2_msg6 = pc2.create_cloud_xyz32(header6, points6)
pc2_msg7 = pc2.create_cloud_xyz32(header7, points7)
pc2_msghand = pc2.create_cloud_xyz32(headerhand, pointshand)
pc2_msgfingerright = pc2.create_cloud_xyz32(headerfinger1, pointsfingerright)
pc2_msgfingerleft = pc2.create_cloud_xyz32(headerfinger2, pointsfingerleft)
# roatation by 90° around x axis
R = ply0.get_rotation_matrix_from_xyz((np.pi/2, 0, 0))
ply0.rotate(R, center=(0, 0, 0))
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
points0 = np.asarray(ply0.points)
points1 = np.asarray(ply1.points)
points2 = np.asarray(ply2.points)
points3 = np.asarray(ply3.points)
points4 = np.asarray(ply4.points)
points5 = np.asarray(ply5.points)
points6 = np.asarray(ply6.points)
points7 = np.asarray(ply7.points)
pointshand = np.asarray(plyhand.points)
pointsfingerright = np.asarray(plyfingerright.points)
pointsfingerleft = np.asarray(plyfingerleft.points)
pc2_msg = pc2.create_cloud_xyz32(header0, points0)
pc2_msg1 = pc2.create_cloud_xyz32(header0, points1)
pc2_msg2 = pc2.create_cloud_xyz32(header0, points2)
pc2_msg3 = pc2.create_cloud_xyz32(header0, points3)
pc2_msg4 = pc2.create_cloud_xyz32(header0, points4)
pc2_msg5 = pc2.create_cloud_xyz32(header0, points5)
pc2_msg6 = pc2.create_cloud_xyz32(header0, points6)
pc2_msg7 = pc2.create_cloud_xyz32(header0, points7)
pc2_msghand = pc2.create_cloud_xyz32(headerhand, pointshand)
pc2_msgfingerright = pc2.create_cloud_xyz32(headerfinger1, pointsfingerright)
pc2_msgfingerleft = pc2.create_cloud_xyz32(headerfinger2, pointsfingerleft)

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    header0.stamp = rospy.Time.now()
    header1.stamp = rospy.Time.now()
    header2.stamp = rospy.Time.now()
    header3.stamp = rospy.Time.now()
    header4.stamp = rospy.Time.now()
    header5.stamp = rospy.Time.now()
    header6.stamp = rospy.Time.now()
    header7.stamp = rospy.Time.now()
    headerhand.stamp = rospy.Time.now()
    headerfinger1.stamp = rospy.Time.now()
    headerfinger2.stamp = rospy.Time.now()
    
    pc2_msg.header = header0
    pc2_msg1.header = header1
    pc2_msg2.header = header2
    pc2_msg3.header = header3
    pc2_msg4.header = header4
    pc2_msg5.header = header5
    pc2_msg6.header = header6
    pc2_msg7.header = header7
    pc2_msghand.header = headerhand
    pc2_msgfingerright.header = headerfinger1
    pc2_msgfingerleft.header = headerfinger2
    pub.publish(pc2_msg)
    pub1.publish(pc2_msg1)
    pub2.publish(pc2_msg2)
    pub3.publish(pc2_msg3)
    pub4.publish(pc2_msg4)
    pub5.publish(pc2_msg5)
    pub6.publish(pc2_msg6)
    pub7.publish(pc2_msg7)
    pubhand.publish(pc2_msghand)
    pubfingerright.publish(pc2_msgfingerright)
    pubfingerleft.publish(pc2_msgfingerleft)

    rate.sleep()
