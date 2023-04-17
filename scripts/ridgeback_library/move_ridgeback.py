"""
Make sure Ridgeback is at QR 1, at depth 0.31m exactly. Check this before even launching
the main script, else everything will go ungu-bunga
"""
import rospy
from geometry_msgs.msg import Twist
import os
import scripts.project_constants
import scripts.intel_realsense_library
import socket

def set_velocity(x: float, y: float, z: float):
    start_time = rospy.Time.now()
    pub = rospy.Publisher(scripts.project_constants.RIDGEBACK_CONTROL_IC_VEL, Twist, queue_size=1)
    while (rospy.Time.now() - start_time).to_sec() < 0.5:
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = 0.0
        twist.angular.x = x - 0.5
        twist.angular.y = y - 0.1
        twist.angular.z = z
        pub.publish(twist)

def command_robot(qr_object: scripts.intel_realsense_library.QR):
    # TODO: Beautify this implementation
    def make_qr_at_center(pixel_mid_x, pixel_mid_y, x, y, z, theta):
        x_vel = 0.0
        y_vel = 0.0
        z_vel = 0.0
        mid_threshold = 20
        if pixel_mid_x > 490:
            y_vel = -0.05
        else:
            y_vel = 0.05
        set_velocity(x_vel, y_vel, z_vel)
        return False

    pixel_mid_x = qr_object.pixel_x
    pixel_mid_y = qr_object.pixel_y
    x = qr_object.x
    y = qr_object.y
    z = qr_object.z
    theta = qr_object.angle
    if make_qr_at_center(pixel_mid_x, pixel_mid_y, x, y, z, theta):
        return True
    else:
        print("Making QR center")
        return False


class RidgebackGoToQR:
    def __init__(self):
        self.current_qr = None  # Always start at QR 1
        self.target_qr = None
        # Ridgeback is basically controlled by Realsense D455
        # therefore intialize the Realsense D455
        self.realsense_d455 = scripts.intel_realsense_library.realsense_get_qr.RealsenseQR(
            scripts.project_constants.D455_REALSENSE_CAMERA_ID
        )
        self.find_initial_qr()

    def find_initial_qr(self):
        self.current_qr = "1"
        for i in ["1", "2", "3", "4", "5"]:
            qr = self.realsense_d455.is_qr_visible(i)
            if qr is not None:
                self.current_qr = i
                break

    def speed_direction(self, target_qr: str):
        if int(self.current_qr) < int(target_qr):
            set_velocity(0.0, 0.5, 0.0)
        elif int(self.current_qr) > int(target_qr):
            set_velocity(0.0, -0.5, 0.0)

    def go_to_qr(self, target_qr: str):
        self.target_qr = target_qr
        # Find if QR is detected
        while 1:
            detected_qr = self.realsense_d455.is_qr_visible(target_qr)
            reached_destination = command_robot(detected_qr)
            if reached_destination:
                break
            else:
                self.speed_direction(target_qr)
        self.current_qr = target_qr
