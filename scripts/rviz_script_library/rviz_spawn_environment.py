import sys
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import tf


def add_box_to_scene(box_pose: geometry_msgs.msg.PoseStamped, l: float, w: float, h: float, box_name):
    scene = moveit_commander.PlanningSceneInterface()
    # The box_pose contains the coordinates of the box which starts at the bottom left corner
    # with length l, width w and height h
    # As Rviz considers the box coordinates to be the center of the box, we need to shift the box_pose
    # by half the length, width and height
    # TODO: Cross Check with Panda Link0 frame
    box_pose.pose.position.x += w / 2
    box_pose.pose.position.y += l / 2
    box_pose.pose.position.z += h / 2

    scene.add_box(box_name, box_pose, size=(w, l, h))
