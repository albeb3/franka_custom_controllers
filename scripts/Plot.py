#!/usr/bin/env python3
import rospy
import tf
import matplotlib.pyplot as plt
import os
import time

if __name__ == "__main__":
    rospy.init_node("plot_ee_node", anonymous=True)
    
    arm_id_left = rospy.get_param("~arm_id_left", "panda_L")
    arm_id_right = rospy.get_param("~arm_id_right", "panda_R")

    listener = tf.TransformListener()

    # Creazione cartella plot se non esiste
    package_path = os.path.dirname(os.path.realpath(__file__))
    plot_path = os.path.join(package_path, "plot")
    if not os.path.exists(plot_path):
        os.makedirs(plot_path)

    # Liste per salvare i dati
    t_data = []
    ee_left_x, ee_left_y, ee_left_z = [], [], []
    ee_right_x, ee_right_y, ee_right_z = [], [], []

    start_time = rospy.Time.now().to_sec()
    rate = rospy.Rate(20.0)  # 20 Hz

    rospy.loginfo("Waiting for TF frames...")

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        elapsed = now - start_time
        if elapsed > 10.0:  # stop after 20 seconds
            rospy.loginfo("20 seconds elapsed, saving plot...")
            break

        try:
            # End-effector left
            (trans_L, rot_L) = listener.lookupTransform(
                arm_id_left+"_link0", arm_id_left+"_EE", rospy.Time(0)
            )
            # End-effector right
            (trans_R, rot_R) = listener.lookupTransform(
                arm_id_right+"_link0", arm_id_right+"_EE", rospy.Time(0)
            )

            t_data.append(elapsed)
            ee_left_x.append(trans_L[0])
            ee_left_y.append(trans_L[1])
            ee_left_z.append(trans_L[2])

            ee_right_x.append(trans_R[0])
            ee_right_y.append(trans_R[1])
            ee_right_z.append(trans_R[2])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rate.sleep()

    # Salva il plot
    plt.figure()
    plt.plot(t_data, ee_left_x, label="Left X")
    plt.plot(t_data, ee_left_y, label="Left Y")
    plt.plot(t_data, ee_left_z, label="Left Z")
    plt.plot(t_data, ee_right_x, label="Right X")
    plt.plot(t_data, ee_right_y, label="Right Y")
    plt.plot(t_data, ee_right_z, label="Right Z")
    plt.xlabel("Time [s]")
    plt.ylabel("Position [m]")
    plt.title("End-effector positions")
    plt.legend()
    plt.grid()

    filename = os.path.join(plot_path, f"ee_positions_{int(time.time())}.png")
    plt.savefig(filename)
    rospy.loginfo(f"Plot saved at: {filename}")
