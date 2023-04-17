import tf
import rospy
from math import pi


def transform_pose_stamped(target_frame, pose_stamped_msg):
    # make a listener
    listener = tf.TransformListener()
    # wait for the transform to be available
    # listener.waitForTransform(target_frame, pose_stamped_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
    listener.waitForTransform(
        target_frame, pose_stamped_msg.header.frame_id, pose_stamped_msg.header.stamp, rospy.Duration(0.5)
    )
    # transform the point
    try:
        transformed_point = listener.transformPose(target_frame, pose_stamped_msg)
        return transformed_point
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Failed to transform point: %s", e)



