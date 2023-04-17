import rospy
import moveit_commander
import tf
import moveit_msgs.msg
from math import pi
import actionlib
import franka_gripper.msg
import franka_msgs.msg
from time import sleep
from multimethod import multimethod
import geometry_msgs.msg
import scripts.tf_scripts
from scripts.tf_scripts.tf_transformation_library import transform_pose_stamped
from sensor_msgs.msg import JointState


class FrankaOperator:
    def __init__(self) -> None:
        """
        This class is used to control the Franka Emika Panda robot.
        Be sure to activate ROS node and initialize moveit_commander before using this class.
        """
        self.robot = moveit_commander.RobotCommander()
        group_name = "panda_manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_planning_time(10.0)
        print(f"============ End effector link: {self.move_group.get_end_effector_link()}")
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20
        )
        self.panda_recovery = rospy.Publisher(
            "/franka_control/error_recovery/goal", franka_msgs.msg.ErrorRecoveryActionGoal, queue_size=20
        )
        self.movegroup_feedback_subscriber = rospy.Subscriber(
            "/move_group/feedback", moveit_msgs.msg.MoveGroupActionFeedback, self._post_execution_status
        )

    def close_gripper(self) -> None:
        client = actionlib.SimpleActionClient("/franka_gripper/grasp", franka_gripper.msg.GraspAction)
        client.wait_for_server()
        goal = franka_gripper.msg.GraspGoal()
        goal.epsilon.inner = 10.0
        goal.epsilon.outer = 10.0
        goal.speed = 0.08
        # goal.force = 50.0
        client.send_goal(goal)
        client.wait_for_result()

    def open_gripper(self) -> None:
        client = actionlib.SimpleActionClient("/franka_gripper/move", franka_gripper.msg.MoveAction)
        client.wait_for_server()
        goal = franka_gripper.msg.MoveGoal()
        goal.width = 0.08
        goal.speed = 0.08
        client.send_goal(goal)
        client.wait_for_result()

    @multimethod
    def move_to_pose(self, target_pose: geometry_msgs.msg.PoseStamped) -> bool:
        waypoints = []
        waypoints.append(target_pose.pose)
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        # Move Group Execute Motion Plan of the robot
        self.move_group.set_pose_target(target_pose)
        execution_status = self.move_group.go(wait=True)
        if not execution_status:
            print("Execution Failed, pausing further execution")
            input("Fix franka errors and press Enter to continue...")
            return self.move_to_pose(target_pose)
        return execution_status

    

