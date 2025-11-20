import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations
import tf2_ros
import tf
import tf2_geometry_msgs
import tf
import numpy as np
import ros_numpy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from reach_space_modeling.srv import ell_params, ell_paramsResponse
from base_optimization.problem_formulation_align_collision import BasePoseOptProblem

from pymoo.algorithms.soo.nonconvex.ga import GA
from pymoo.algorithms.soo.nonconvex.pso import PSO
from pymoo.algorithms.soo.nonconvex.de import DE
from pymoo.algorithms.soo.nonconvex.pattern import PatternSearch
from pymoo.algorithms.soo.nonconvex.es import ES
from pymoo.optimize import minimize
from pymoo.termination.robust import RobustTermination
from pymoo.termination.ftol import SingleObjectiveSpaceTermination

from base_optimization.srv import octomap2cloud, octomap2cloudResponse
import moveit_commander

def move_arm():
    # set the target
    rospy.loginfo("Setting the robot arm's target...")
    end_effector_link = robot_arm.get_end_effector_link()
    robot_arm.set_pose_reference_frame("map")
    robot_arm.set_start_state_to_current_state()
    robot_arm.set_pose_target(des_pose_msg, end_effector_link)
    
    success = None
    while not success:
        # start moving and wait for termination
        rospy.loginfo("Starting the movements...")
        success = robot_arm.go(wait=True)
        if success:
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.loginfo("Failed!")
           
    # clear residual movements
    rospy.loginfo("Cleaning residual movements...")
    robot_arm.stop()
    
    # remove target poses
    rospy.loginfo("Cleaning targets...")
    robot_arm.clear_pose_targets()
    
def send_opt_base_pose(x_base, y_base, theta_base):
    # received base_pose is a PoseStmaped msg
    # create a MoveBaseGoal message
    goal_base_pose = MoveBaseGoal()
    goal_base_pose.target_pose.header.stamp = rospy.Time.now()
    goal_base_pose.target_pose.header.frame_id = 'map'
        
    goal_base_pose.target_pose.pose.position.x = x_base
    goal_base_pose.target_pose.pose.position.y = y_base
    goal_base_pose.target_pose.pose.position.z = 0

    quat = tf.transformations.quaternion_from_euler(
        0, 0, np.deg2rad(theta_base), axes='sxyz')
    goal_base_pose.target_pose.pose.orientation.x = quat[0]
    goal_base_pose.target_pose.pose.orientation.y = quat[1]
    goal_base_pose.target_pose.pose.orientation.z = quat[2]
    goal_base_pose.target_pose.pose.orientation.w = quat[3]

    # publish the goal message
    rospy.loginfo("Moving the base to the optimal pose...")
    move_base_client.send_goal(goal_base_pose)
    
    rospy.sleep(0.5)
    
    # wait for reaching the goal
    state = move_base_client.get_state()
    while state != actionlib.GoalStatus.SUCCEEDED:
        rospy.sleep(0.5)
        state = move_base_client.get_state()
    
    rospy.loginfo("Base pose reached successfully")
        
def find_opt_base_pose(ell_frame_link, des_pose, point_cloud):
    rospy.loginfo("Looking for an optimal base pose...")
    # define the optimization problem to find the optimal base pose
    problem = BasePoseOptProblem(
        ell_center_map, ell_axis_out, ell_axis_inn, des_pose, point_cloud)

    # solve the optimization problem
    algorithm = PSO()
    termination = RobustTermination(
        SingleObjectiveSpaceTermination(tol=pow(10, -4))
    )

    # solve the optimization problem
    res = minimize(problem=problem,
                   algorithm=algorithm,
                   termination=termination,
                   verbose=False,
                   seed=1)

    rospy.loginfo("Optimal base pose: x=%.4f, y=%.4f, theta=%.4f",
                  res.X[0], res.X[1], res.X[2])
    rospy.loginfo("Solution found in %.4f s", res.exec_time)
    print(res.F)

    # Here I obtain the position of the ellipsoid with respect to R0
    # I need to find the coordinates of the base that allow me
    # to place the center of the ellipsoid at the desired point

    tmp = PoseStamped()
    tmp.header.frame_id = "map"
    tmp.header.stamp = rospy.Time.now()

    tmp.pose.position.x = res.X[0]
    tmp.pose.position.y = res.X[1]
    tmp.pose.position.z = ell_center_map[2]

    quat = tf.transformations.quaternion_from_euler(
        0, 0, np.deg2rad(res.X[2]), axes='sxyz')
    tmp.pose.orientation.x = quat[0]
    tmp.pose.orientation.y = quat[1]
    tmp.pose.orientation.z = quat[2]
    tmp.pose.orientation.w = quat[3]

    tmp_pub.publish(tmp)

    homog_matr = np.zeros((4, 4))
    homog_matr[:3, :3] = Rotation.from_euler(
        'xyz', [0, 0, res.X[2]], degrees=True).as_matrix()
    homog_matr[:3, 3] = np.array([res.X[0], res.X[1], 0])
    homog_matr[3, 3] = 1
    base_pos = np.dot(homog_matr, np.array(
        [-ell_center_base[0], -ell_center_base[1], 0, 1]))
    base_pos[2] = res.X[2]


    send_opt_base_pose(base_pos[0], base_pos[1], base_pos[2])

def handle_des_EE_pose(data):
    global des_pose_msg
    des_pose_msg = data.pose
    
    des_position = np.array(
        [data.pose.position.x, data.pose.position.y, data.pose.position.z])

    des_orientation = tf.transformations.euler_from_quaternion(np.array(
        [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]))
    des_orientation = np.rad2deg(des_orientation)

    des_pose = np.concatenate((des_position, des_orientation))

    tmp_pub.publish(data)

    # request the conversion from octomap to point cloud
    rospy.loginfo(
        "Waiting for service '/locobot/octomap2cloud_converter_srv' ...")
    rospy.wait_for_service("/locobot/octomap2cloud_converter_srv")
    octomap2cloud_srv = rospy.ServiceProxy(
        "/locobot/octomap2cloud_converter_srv", octomap2cloud)

    rospy.loginfo(
        "Sending a service request to '/locobot/octomap2cloud_converter_srv'...")
    occ_cloud = octomap2cloud_srv()
    rospy.loginfo("Response received")

    # convert the PointCloud2 message into a umpy array
    cloud_np = ros_numpy.numpify(occ_cloud.cloud)

    # find the optimal base pose
    find_opt_base_pose(ell_ref_frame, des_pose, cloud_np)
    
    # move the arm
    # move_arm()


# create a ROS node
rospy.init_node('find_opt_pose', anonymous=True)

# # wait for gazebo to be unpaued
# rospy.wait_for_service("/gazebo/get_physics_properties")

# get_physics = rospy.ServiceProxy(
#     "/gazebo/get_physics_properties", GetPhysicsProperties)
# rospy.loginfo("Waiting for Gazebo to be unpaused...")

# while not rospy.is_shutdown():
#     try:
#         physics = get_physics()
#         if physics.pause != True:  # Gazebo is unpaused if gravity is nonzero
#             rospy.loginfo("Gazebo unpaused! Proceeding...")
#             break
#     except rospy.ServiceException:
#         pass  # If service is not available, keep trying

#     rospy.sleep(1)

# retrieve the parameter of the ellipsoid
rospy.loginfo("Waiting for serive /get_ellipsoid_params...")
rospy.wait_for_service('get_ellipsoid_params')
ell_params_srv = rospy.ServiceProxy('get_ellipsoid_params', ell_params)
rospy.loginfo("Service /get_ellipsoid_params is available")

rospy.loginfo("Sending request to /get_ellipsoid_params...")
ell_par = ell_params_srv()
rospy.loginfo("Ellipsoid parameters received: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
              ell_par.xC, ell_par.yC, ell_par.zC,
              ell_par.aO, ell_par.bO, ell_par.cO,
              ell_par.aI, ell_par.bI, ell_par.cI)
rospy.loginfo("Ellipsoid reference frame: %s", ell_par.ell_ref_frame)

ell_center = np.array([ell_par.xC, ell_par.yC, ell_par.zC])
ell_axis_out = np.array([ell_par.aO, ell_par.bO, ell_par.cO])
ell_axis_inn = np.array([ell_par.aI, ell_par.bI, ell_par.cI])
ell_ref_frame = ell_par.ell_ref_frame

# transform the center from ell_ref_fram to the fixed frame ("map")
center_pose = PoseStamped()
center_pose.header.stamp = rospy.Time.now()
center_pose.header.frame_id = ell_ref_frame

center_pose.pose.position.x = ell_center[0]
center_pose.pose.position.y = ell_center[1]
center_pose.pose.position.z = ell_center[2]

center_pose.pose.orientation.x = 0.0
center_pose.pose.orientation.y = 0.0
center_pose.pose.orientation.z = 0.0
center_pose.pose.orientation.w = 1.0

    
base_pose = PoseStamped()
base_pose.header.stamp = rospy.Time.now()
base_pose.header.frame_id = "locobot/base_footprint"

base_pose.pose.position.x = 0
base_pose.pose.position.y = 0
base_pose.pose.position.z = 0

base_pose.pose.orientation.x = 0
base_pose.pose.orientation.y = 0
base_pose.pose.orientation.z = 0
base_pose.pose.orientation.w = 1

ell_center_pub = rospy.Publisher("/ell_center", PoseStamped, queue_size=10)
base_center_pub = rospy.Publisher("/base_center_pub", PoseStamped, queue_size=10)
for i in range(5):
    rospy.sleep(1)
    base_center_pub.publish(base_pose)
    rospy.sleep(1)
    ell_center_pub.publish(center_pose)



tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

tf_buffer.can_transform("map", ell_ref_frame, rospy.Time(0))
transform = tf_buffer.lookup_transform(
    "map", ell_ref_frame, rospy.Time(0), rospy.Duration(2))
ell_transformed_msg = tf2_geometry_msgs.do_transform_pose(center_pose, transform)

ell_center_map = np.array([ell_transformed_msg.pose.position.x,
                           ell_transformed_msg.pose.position.y,
                           ell_transformed_msg.pose.position.z])

# the reference frame attacched to the center of the ellipsoid
# is fixed with respect to the reference frame of the mobile base
# for what concern both the orientation and the position.
# here the translation vector between these two ref. frames is built
tf_buffer.can_transform("locobot/base_footprint", ell_ref_frame, rospy.Time(0))
transform = tf_buffer.lookup_transform(
    "locobot/base_footprint", ell_ref_frame, rospy.Time(0), rospy.Duration(1))
ell_transformed_msg = tf2_geometry_msgs.do_transform_pose(center_pose, transform)
ell_center_base = np.array([ell_transformed_msg.pose.position.x,
                            ell_transformed_msg.pose.position.y,
                            ell_transformed_msg.pose.position.z])

des_pose_msg = None
rospy.Subscriber("/des_EE_pose", PoseStamped, callback=handle_des_EE_pose)
tmp_pub = rospy.Publisher("/des_EE_pose_tmp", PoseStamped, queue_size=10)

rospy.sleep(0.5)

# use the actionlib to publish the base goal
move_base_client = actionlib.SimpleActionClient("/locobot/move_base", MoveBaseAction)
move_base_client.wait_for_server()

# configure the moveit interface
moveit_commander.roscpp_initialize('joint_states:=/locobot/joint_states')
robot_arm = moveit_commander.MoveGroupCommander("interbotix_arm", robot_description='/locobot/robot_description', ns="/locobot", wait_for_servers=20)

print()
rospy.loginfo("Waiting for the desired end-effector pose...")
rospy.spin()
