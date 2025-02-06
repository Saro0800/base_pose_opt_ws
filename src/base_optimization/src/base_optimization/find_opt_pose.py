import octomap_msgs.msg
import rospy
import tf.transformations
import tf2_ros
import tf
import tf2_geometry_msgs
import tf
import numpy as np
import ros_numpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from sensor_msgs.msg import PointCloud2

from reach_space_modeling.generate_pointcloud.gen_cloud_GUI import GenereatePointCloud
from reach_space_modeling.opt_problem.eqn_solv_opt import solve_eqn_prob
from reach_space_modeling.srv import ell_params, ell_paramsResponse
from base_optimization.problem_formulation_align import BasePoseOptProblem

from pymoo.algorithms.soo.nonconvex.pso import PSO
from pymoo.optimize import minimize
from pymoo.termination.robust import RobustTermination
from pymoo.termination.ftol import SingleObjectiveSpaceTermination

from base_optimization.srv import octomap2cloud, octomap2cloudResponse


def send_opt_base_pose(x_base, y_base, theta_base):
    # received base_pose is a PoseStmaped msg

    # create a MoveBaseActionGoal message
    goal_base_pose = MoveBaseActionGoal()

    goal_base_pose.goal.target_pose.header.stamp = rospy.Time.now()
    goal_base_pose.goal.target_pose.header.frame_id = 'map'

    goal_base_pose.goal.target_pose.pose.position.x = x_base
    goal_base_pose.goal.target_pose.pose.position.y = y_base
    goal_base_pose.goal.target_pose.pose.position.z = 0

    quat = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(theta_base), axes='sxyz')
    goal_base_pose.goal.target_pose.pose.orientation.x = quat[0]
    goal_base_pose.goal.target_pose.pose.orientation.y = quat[1]
    goal_base_pose.goal.target_pose.pose.orientation.z = quat[2]
    goal_base_pose.goal.target_pose.pose.orientation.w = quat[3]

    # publish the goal message
    rospy.loginfo("Moving the base to the optimal pose...")
    move_base_pub.publish(goal_base_pose)


def find_opt_base_pose(ell_frame_link, des_pose):
    rospy.loginfo("Looking for an optimal base pose...")
    # define the optimization problem to find the optimal base pose
    problem = BasePoseOptProblem(ell_center_map, ell_axis, des_pose)

    # solve the optimization problem using the PSO algorithm
    algorithm = PSO()
    termination = RobustTermination(
        SingleObjectiveSpaceTermination(tol=pow(10, -6))
    )

    # solve the optimization problem
    res = minimize(problem=problem,
                   algorithm=algorithm,
                   termination=termination,
                   verbose=False,
                   seed=1)

    rospy.loginfo("Optimal base pose: x=%.4f, y=%.4f, theta=%.4f",
                  res.X[0], res.X[1], res.X[2])    
    print(res.F)
    
    # io qui ottengo la posizione dell'ellissoide rispetto a R0
    # devo trovare le coordinate della base che mi permettono di arrivare
    # ad avere il centro dell'ellissoide nel punto desiderato
    
    tmp = PoseStamped()
    tmp.header.frame_id = "map"
    tmp.header.stamp = rospy.Time.now()
    
    tmp.pose.position.x = res.X[0]
    tmp.pose.position.y = res.X[1]
    tmp.pose.position.z = ell_center_map[2]
    
    quat = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(res.X[2]), axes='sxyz')
    tmp.pose.orientation.x = quat[0]
    tmp.pose.orientation.y = quat[1]
    tmp.pose.orientation.z = quat[2]
    tmp.pose.orientation.w = quat[3]
    
    tmp_pub.publish(tmp)
    
    send_opt_base_pose(res.X[0], res.X[1], res.X[2])


def handle_des_EE_pose(data):

    des_position = np.array(
        [data.pose.position.x, data.pose.position.y, data.pose.position.z])

    des_orientation = tf.transformations.euler_from_quaternion(np.array(
        [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]))
    des_orientation = np.rad2deg(des_orientation)
    print(des_orientation)
    des_pose = np.concatenate((des_position, des_orientation))

    tmp_pub.publish(data)
    
    # request the conversion from octomap to point cloud
    rospy.loginfo("Waiting for service '/locobot/octomap2cloud_converter_srv' ...")
    rospy.wait_for_service("/locobot/octomap2cloud_converter_srv")
    octomap2cloud_srv = rospy.ServiceProxy("/locobot/octomap2cloud_converter_srv", octomap2cloud)
    
    rospy.loginfo("Sending a service request to '/locobot/octomap2cloud_converter_srv'...")
    occ_cloud = octomap2cloud_srv()
    rospy.loginfo("Response received")

    # convert the PointCloud2 message into a umpy array
    cloud_np = ros_numpy.numpify(occ_cloud.cloud)
    
    print(cloud_np.shape)
    
    
    assert(False)
    

    # find the optimal base pose
    find_opt_base_pose(ell_ref_frame, des_pose)


# create a ROS node
rospy.init_node('find_opt_pose', anonymous=True)

# retrieve the parameter of the ellipsoid
rospy.loginfo("Waiting for serive /get_ellipsoid_params...")
rospy.wait_for_service('get_ellipsoid_params')
ell_params_srv = rospy.ServiceProxy('get_ellipsoid_params', ell_params)
rospy.loginfo("Service /get_ellipsoid_params is available")

rospy.loginfo("Sending request to /get_ellipsoid_params...")
ell_par = ell_params_srv()
rospy.loginfo("Ellipsoid parameters received: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
              ell_par.xC, ell_par.yC, ell_par.zC, ell_par.a, ell_par.b, ell_par.c)
rospy.loginfo("Ellipsoid reference frame: %s", ell_par.ell_ref_frame)

ell_center = np.array([ell_par.xC, ell_par.yC, ell_par.zC])
ell_axis = np.array([ell_par.a, ell_par.b, ell_par.c])
ell_ref_frame = ell_par.ell_ref_frame

# transform the center from ell_ref_fram to the fixed frame ("map")
tmp_pose = PoseStamped()
tmp_pose.header.stamp = rospy.Time.now()
tmp_pose.header.frame_id = ell_ref_frame

tmp_pose.pose.position.x = ell_center[0]
tmp_pose.pose.position.y = ell_center[1]
tmp_pose.pose.position.z = ell_center[2]

tmp_pose.pose.orientation.x = 0.0
tmp_pose.pose.orientation.y = 0.0
tmp_pose.pose.orientation.z = 0.0
tmp_pose.pose.orientation.w = 1.0

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

tf_buffer.can_transform("map", ell_ref_frame, rospy.Time(0))
transform = tf_buffer.lookup_transform("map", ell_ref_frame, rospy.Time(0), rospy.Duration(1))
ell_transformed_msg = tf2_geometry_msgs.do_transform_pose(tmp_pose, transform)

ell_center_map = np.array([ell_transformed_msg.pose.position.x,
                           ell_transformed_msg.pose.position.y,
                           ell_transformed_msg.pose.position.z])

rospy.Subscriber("/des_EE_pose", PoseStamped, callback=handle_des_EE_pose)
tmp_pub = rospy.Publisher("/des_EE_pose_tmp", PoseStamped, queue_size=10)

rospy.sleep(0.5)

# publish on topic to move the base
move_base_pub = rospy.Publisher("/locobot/move_base/goal", MoveBaseActionGoal, queue_size=10)

print()
rospy.loginfo("Waiting for the desired end-effector pose...")
rospy.spin()
