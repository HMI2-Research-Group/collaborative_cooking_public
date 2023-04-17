import sys
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import tf
import random


def add_3d_obj_to_scene(
    obj_original_tf_frame: str, box_pose: geometry_msgs.msg.PoseStamped, obj_mesh: str, obj_name="box"
):
    # get full path of alexandros_robot catkin package
    mesh_obj_path = rospkg.RosPack().get_path("alexandros_robot") + "/meshes/" + obj_mesh
    scene = moveit_commander.PlanningSceneInterface()
    listener = tf.TransformListener()
    listener.waitForTransform("panda_link0", obj_original_tf_frame, rospy.Time(), rospy.Duration(1.0))

    try:
        transformed_point = listener.transformPose("panda_link0", box_pose)
        transformed_point.pose.orientation.x = 0.0
        transformed_point.pose.orientation.y = 0.0
        transformed_point.pose.orientation.z = 0.0
        transformed_point.pose.orientation.w = 1.0
        # my_mesh_marker = Marker()
        # my_mesh_marker.header.frame_id = "panda_link0"
        # my_mesh_marker.type = my_mesh_marker.MESH_RESOURCE
        # my_mesh_marker.mesh_resource = mesh_obj_path
        # my_mesh_marker.mesh_use_embedded_materials = False
        # my_mesh_marker.pose = transformed_point.pose
        # my_mesh_marker.scale.x = 1.0
        # my_mesh_marker.scale.y = 1.0
        # my_mesh_marker.scale.z = 1.0
        # my_mesh_marker.color.r = random.randint(0, 255)
        # my_mesh_marker.color.g = random.randint(0, 255)
        # my_mesh_marker.color.b = random.randint(0, 255)
        # my_mesh_marker.color.a = 1.0

        # my_mesh_marker.lifetime = rospy.Duration(10.0)
        # my_mesh_marker.action = my_mesh_marker.ADD
        # my_mesh_marker.id = 0
        # my_mesh_marker.ns = "my_mesh_marker"
        # my_mesh_marker.header.stamp = rospy.Time.now()
        # my_mesh_marker_pub = rospy.Publisher("move_group/monitored_planning_scene", Marker, queue_size=10)
        # my_mesh_marker_pub.publish(my_mesh_marker)
        box_name = obj_name
        scene.add_mesh(
            box_name,
            transformed_point,
            mesh_obj_path,
        )

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Failed to transform point: %s", e)


def remove_3d_obj_from_scene(obj_name: str):
    scene = moveit_commander.PlanningSceneInterface()
    # Get all objects in the scene
    for obj_name in scene.get_known_object_names():
        if obj_name == "Walls":
            continue
        scene.remove_world_object(obj_name)
