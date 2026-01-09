#!/usr/bin/env python3

from __future__ import annotations
import rospy 
from geometry_msgs.msg import TransformStamped
from typing import *
import sys
import threading

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from PyQt5 import QtWidgets
import math
import tf2_ros
import geometry_msgs.msg
import std_msgs.msg

# Variabili aggiornate dal callback
time  = 0.0
x_val = 0.0
y_val = 0.0
z_val = 0.0

# Liste per registrare i valori ai trigger
x_start = []
y_start = []
z_start = []
x_finish = []
y_finish = []
z_finish = []

# Flag per disegnare linee verticali
trigger_start = False
trigger_finish = False
trigger_start_indices = [0.0]*200
trigger_finish_indices = [0.0]*200



class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.setGeometry(300, 300, 800, 400)
        self.setWindowTitle("Plot live TF")

        self.frm = QtWidgets.QFrame(self)
        self.lyt = QtWidgets.QVBoxLayout()
        self.frm.setLayout(self.lyt)
        self.setCentralWidget(self.frm)

        self.myFig = MyFigureCanvas(time=200,x_range=[-2, 2], y_range=[-2, 2], z_range=[-2, 2], interval=30)
        self.lyt.addWidget(self.myFig)

        self.show()


class MyFigureCanvas(FigureCanvas):

    def __init__(self, time:int,x_range:List, y_range:List, z_range:List, interval:int):
        fig = Figure()
        super().__init__(fig)

        self._time_len = time
        self._x_range = x_range
        self._y_range = y_range
        self._z_range = z_range

        self._time = list(range(0, self._time_len))

        self._x = [0.0] * self._time_len
        self._y = [0.0] * self._time_len
        self._z = [0.0] * self._time_len

        self._ax = self.figure.subplots()

        # Timer per aggiornare la figura
        self._timer = self.new_timer(interval)
        self._timer.add_callback(self.update_plot)
        self._timer.start()

    def update_plot(self):
        global x_val, y_val, z_val, trigger_start, trigger_finish

        # Aggiorna buffer segnali
        self._x.append(x_val)
        self._x = self._x[-self._time_len:]  
        self._y.append(y_val)
        self._y = self._y[-self._time_len:]  
        self._z.append(z_val)
        self._z = self._z[-self._time_len:]     
        # Linee verticali ai trigger
        if trigger_start:
            trigger_start_indices.append(len(self._time) - 1)
            trigger_start = False
        if trigger_start_indices:
            trigger_start_indices[:]=trigger_start_indices[-self._time_len:]
        
        if trigger_finish:
            trigger_finish_indices.append(len(self._time) - 1)
            trigger_finish = False   
        if trigger_finish_indices:
            trigger_finish_indices[:]=trigger_finish_indices[-self._time_len:]       
        self._ax.clear()

        # Disegna segnali
        self._ax.plot(self._time, self._x, label='X', color='r')
        self._ax.plot(self._time, self._y, label='Y', color='g')
        self._ax.plot(self._time, self._z, label='Z', color='b')

        
        for idx in trigger_start_indices:
            self._ax.axvline(x=idx, color='r', linestyle='--' )
        

        for idx in trigger_finish_indices:
            self._ax.axvline(x=idx, color='g', linestyle='--')
        self._ax.set_ylim(self._y_range[0], self._y_range[1])
        self._ax.set_title('Relative position between left_hand and right_hand')
        self._ax.set_xlabel('Time (samples)')
        self._ax.set_ylabel('Position (m)')
        self._ax.legend(loc='upper right')

        self.draw()


def start_qt_gui():
    app = QtWidgets.QApplication(sys.argv)
    window = ApplicationWindow()
    app.exec_()


def callback(msg):
    global trigger_start, trigger_finish
    global x_start, y_start, z_start, x_finish, y_finish, z_finish
    global x_val, y_val, z_val

    if msg.data is True:
        trigger_start = True
        x_start.append(x_val)
        y_start.append(y_val)
        z_start.append(z_val)

    elif msg.data is False:
        trigger_finish = True
        x_finish.append(x_val)
        y_finish.append(y_val)
        z_finish.append(z_val)



if __name__ == "__main__":

    # 1) ROS nel thread principale
    rospy.init_node("transform_listener", anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.Subscriber("/trigger/left_hand", std_msgs.msg.Bool, callback)

    # 2) Avvio GUI in thread separato
    gui_thread = threading.Thread(target=start_qt_gui)
    gui_thread.daemon = True
    gui_thread.start()

    rate = rospy.Rate(20.0) 

    # Loop principale
    while not rospy.is_shutdown():
        try: 
            trans = tfBuffer.lookup_transform('left_hand', 'right_hand', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        x_val = trans.transform.translation.x
        y_val = trans.transform.translation.y
        z_val = trans.transform.translation.z

        print(f"Distance - x: {x_val:.2f}, y: {y_val:.2f}, z: {z_val:.2f}")
        rate.sleep()
        
    rospy.spin()
