import rospy
from geometry_msgs.msg import PoseStamped
import tf
import tf.transformations
import numpy as np

# create a rosnode
rospy.init_node("des_EE_pose_publisher", anonymous=True)

# create a PoseStamped message
des_EE_pose = PoseStamped()

des_EE_pose.header.frame_id = "map"
des_EE_pose.header.stamp = rospy.Time.now()

des_EE_pose.pose.position.x = -3.5
des_EE_pose.pose.position.y = 2.75
des_EE_pose.pose.position.z = 0.4

des_orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(179), axes='sxyz')
des_EE_pose.pose.orientation.x = des_orientation[0]
des_EE_pose.pose.orientation.y = des_orientation[1]
des_EE_pose.pose.orientation.z = des_orientation[2]
des_EE_pose.pose.orientation.w = np.ma([0.0001, des_orientation[3]])

# create a publisher to publish the desired end-effector pose
des_EE_topic = rospy.Publisher("/des_EE_pose", PoseStamped, queue_size=10)

rospy.loginfo("Publishing the desired end-effector pose...")
# publish the desired end-effector pose
rospy.sleep(1)
des_EE_topic.publish(des_EE_pose)
# rospy.spin()
# while not rospy.is_shutdown():
#     rate.sleep()
