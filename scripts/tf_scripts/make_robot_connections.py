import tf
import rospy
import math
from math import pi


def main():
    # publish a tf frame connecting two frames
    br = tf.TransformBroadcaster()
    rospy.init_node("make_robot_connections")
    rate = rospy.Rate(100.0)
    # dist_r = 0.068 / math.sqrt(2)
    dist_r = 0.1 / math.sqrt(2)
    # make it flotaing point till three decimal places
    dist_r = float("{:.4f}".format(dist_r))
    while not rospy.is_shutdown():
        br.sendTransform(
            (0.13, 0.0, -0.134),
            tf.transformations.quaternion_from_euler(0, 0, pi / 2.0),
            rospy.Time.now(),
            "camera_color_optical_frame",
            "panda_hand_tcp",
        )
        rate.sleep()


if __name__ == "__main__":
    main()
