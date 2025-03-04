import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def add_new_pose(data):
    rospy.loginfo("received a new optimal base pose...")
    opt_base_poses.append(data)


# create a new ros node
rospy.init_node("move_to_next_pose")
rospy.loginfo("Created node 'move_to_next_pose'")

# subscribe to a topic to receive the newly found pose for the mobile base
rospy.Subscriber("add_opt_base_pose", PoseStamped, add_new_pose)
rospy.loginfo("Created topic 'add_opt_base_pose'")

# publish on topic to move the base
move_base_client = actionlib.SimpleActionClient("/locobot/move_base", MoveBaseAction)
move_base_client.wait_for_server()

# create a list of poses (queue)
opt_base_poses = []

rospy.loginfo("Waiting for a new optimal base pose...")
# start looping on all the desired poses
while True:
    # check for empy
    if len(opt_base_poses)<=0:
        rospy.sleep(2)
    else:
        rospy.sleep(2)
        
        # extract the first element (oldest inserted)
        nxt_pose = opt_base_poses.pop(0)
        
        # create a MoveBaseGoal message
        goal_base_pose = MoveBaseGoal()

        goal_base_pose.target_pose.header.stamp = rospy.Time.now()
        goal_base_pose.target_pose.header.frame_id = 'map'
        
        goal_base_pose.target_pose.pose = nxt_pose.pose
        
        # retry if fail
        res = False
        while res == False:
            # publish the goal message
            rospy.loginfo("Moving the base to the optimal pose...")
            move_base_client.send_goal(goal_base_pose)

            # wait for ending
            res = move_base_client.wait_for_result()
    


