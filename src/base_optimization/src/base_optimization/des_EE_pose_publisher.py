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

# # pose 1
des_EE_pose.pose.position.x = 3.034
des_EE_pose.pose.position.y = -3.684
des_EE_pose.pose.position.z = 0.39
des_orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(195), axes='sxyz')
# # pose 2
# des_EE_pose.pose.position.x = 2.875
# des_EE_pose.pose.position.y = -3.414
# des_EE_pose.pose.position.z = 0.39
# des_orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(275), axes='sxyz')
# # # pose 3
# des_EE_pose.pose.position.x = 2.358
# des_EE_pose.pose.position.y = -2.532
# des_EE_pose.pose.position.z = 0.48
# des_orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(135), axes='sxyz')
# # pose 4
# des_EE_pose.pose.position.x = 1.758
# des_EE_pose.pose.position.y = -2.325
# des_EE_pose.pose.position.z = 0.48
# des_orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(0), axes='sxyz')
# # pose 5
# des_EE_pose.pose.position.x = 2.038
# des_EE_pose.pose.position.y = -2.121
# des_EE_pose.pose.position.z = 0.48
# des_orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(275), axes='sxyz')
# # pose 6
# des_EE_pose.pose.position.x = 1.973
# des_EE_pose.pose.position.y = -1.350
# des_EE_pose.pose.position.z = 0.39
# des_orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(90), axes='sxyz')
# # # pose 7
# des_EE_pose.pose.position.x = 2.080
# des_EE_pose.pose.position.y = -1.048
# des_EE_pose.pose.position.z = 0.39
# des_orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(205), axes='sxyz')
# # # pose 8
# des_EE_pose.pose.position.x = 1.480
# des_EE_pose.pose.position.y = -0.449
# des_EE_pose.pose.position.z = 0.29
# des_orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(195), axes='sxyz')
# # # pose 9
# des_EE_pose.pose.position.x = 1.165
# des_EE_pose.pose.position.y = -0.160
# des_EE_pose.pose.position.z = 0.29
# des_orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(275), axes='sxyz')
# # # pose 10
# des_EE_pose.pose.position.x = 0.817
# des_EE_pose.pose.position.y = -0.435
# des_EE_pose.pose.position.z = 0.29
# des_orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(0), axes='sxyz')

# # pose 1 (optimal value 0.0)
# des_EE_pose.pose.position.x = 2
# des_EE_pose.pose.position.y = 0.5
# des_EE_pose.pose.position.z = 0.4
# des_orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(0), axes='sxyz')

# pose 2
# des_EE_pose.pose.position.x = 3.034
# des_EE_pose.pose.position.y = -3.684
# des_EE_pose.pose.position.z = 0.39
# des_orientation = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(195), axes='sxyz')



des_EE_pose.pose.orientation.x = des_orientation[0]
des_EE_pose.pose.orientation.y = des_orientation[1]
des_EE_pose.pose.orientation.z = des_orientation[2]
des_EE_pose.pose.orientation.w = des_orientation[3]

# create a publisher to publish the desired end-effector pose
des_EE_topic = rospy.Publisher("/des_EE_pose", PoseStamped, queue_size=10)

rospy.loginfo("Publishing the desired end-effector pose...")
# publish the desired end-effector pose
rospy.sleep(1)
des_EE_topic.publish(des_EE_pose)
# rospy.spin()
# while not rospy.is_shutdown():
#     rate.sleep()
