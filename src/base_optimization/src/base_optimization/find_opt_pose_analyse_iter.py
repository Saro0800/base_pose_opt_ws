import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations
import tf2_ros
import tf
import tf2_geometry_msgs
import tf
import numpy as np
import ros_numpy
from geometry_msgs.msg import PoseStamped

from reach_space_modeling.srv import ell_params, ell_paramsResponse
# from base_optimization.problem_formulation_align import BasePoseOptProblem
from base_optimization.problem_formulation_align_collision import BasePoseOptProblem

from pymoo.algorithms.soo.nonconvex.ga import GA
from pymoo.algorithms.soo.nonconvex.pso import PSO
from pymoo.algorithms.soo.nonconvex.de import DE
from pymoo.algorithms.soo.nonconvex.pattern import PatternSearch
from pymoo.algorithms.soo.nonconvex.es import ES

from pymoo.optimize import minimize
from pymoo.termination.robust import RobustTermination
from pymoo.termination.ftol import SingleObjectiveSpaceTermination
from pymoo.operators.mutation.pm import PM
from pymoo.operators.crossover.sbx import SBX

from base_optimization.srv import octomap2cloud, octomap2cloudResponse
from gazebo_msgs.srv import GetPhysicsProperties
import matplotlib.pyplot as plt

from tqdm import tqdm
import h5py


def find_opt_base_pose(ell_frame_link, des_pose, point_cloud, algorithm, n_gen=100):
    # rospy.loginfo("Looking for an optimal base pose...")
    # define the optimization problem to find the optimal base pose
    problem = BasePoseOptProblem(
        ell_center_map, ell_axis_out, ell_axis_inn, des_pose, point_cloud)

    # solve the optimization problem using the PSO algorithm
    termination = RobustTermination(
        SingleObjectiveSpaceTermination(tol=pow(10, -3))
    )

    # solve the optimization problem
    if n_gen is None:
        res = minimize(problem=problem,
                    algorithm=algorithm,
                    termination=termination,
                    verbose=False,
                    seed=1)
    else:
        res = minimize(problem=problem,
                    algorithm=algorithm,
                    termination=('n_gen', n_gen),
                    verbose=False,
                    seed=1)
        

    # rospy.loginfo("Optimal base pose: x=%.4f, y=%.4f, theta=%.4f",
    #               res.X[0], res.X[1], res.X[2])
    # rospy.loginfo("Solution found in %.4f s", res.exec_time)
    # print(res.F)

    return res


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


    # find the optimal base pose using the GA (50 times)
    n_times = 25

    
    for alg_name in ["PSO"]:
    
        for n_gen in [200]:
            f_val = []
            time_val = []
        
            for i in tqdm(range(n_times),
                        total=n_times,
                        desc="Problem solved using {:} ({:} generations)".format(alg_name, n_gen),
                        ncols=90):
                
                pop_size = 15
                init_pop = np.zeros((pop_size, 3))
                init_pop[:, :2] = np.random.uniform(-5, 5, (pop_size, 2))
                init_pop[:, 2] = np.random.uniform(0, 360, (pop_size,))

                if alg_name == "GA":
                    sbx = SBX(prob=1.0, prob_var=1.0)
                    pm = PM(prob=0.01)
                    
                    algorithm = GA(pop_size=pop_size,
                                save_history=True,
                                sampling=init_pop,
                                crossover=sbx,
                                mutation=pm)

                elif alg_name == "PSO":
                    algorithm = PSO(pop_size=pop_size,
                                    save_history=True,
                                    sampling=init_pop,
                                    adaptive=False,
                                    w=0.9,
                                    c1=0.5,
                                    c2=2.0)

                elif alg_name == "ES":
                    algorithm = ES(pop_size=pop_size,
                                save_history=True,
                                sampling=init_pop)

                elif alg_name == "DE":
                    algorithm = DE(pop_size=pop_size,
                                save_history=True,
                                sampling=init_pop)

                elif alg_name == "PS":
                    algorithm = PatternSearch(pop_size=15,
                                            save_history=True,
                                            sampling=init_pop)

                res = find_opt_base_pose(ell_ref_frame, des_pose, cloud_np, algorithm, n_gen)

                f_val.append([algo.opt.get("F").min() for algo in res.history])
                time_val.append(res.exec_time)
                # res_dict_x[alg_name].append([algo.opt.get("X")[np.argmin(algo.opt.get("F"))] for algo in res.history])
            
            if n_gen is None:
                file_name = "{:}_pose2_noGen_25init.hdf5".format(alg_name)
            else:
                file_name = "{:}_pose2_{:}gen_25init_pop15_w09_c105_c22.hdf5".format(alg_name, n_gen)
    
            hfile = h5py.File(file_name, 'w')
            
            hfile.create_dataset(alg_name+"_F", data=f_val)
            hfile.create_dataset(alg_name+"_time", data=time_val)
            # hfile.create_dataset(alg_name+"_x", data=res_dict_x[alg_name])
    rospy.loginfo("Problem solved using the {:} algorithm".format(alg_name))


# create a ROS node
rospy.init_node('find_opt_pose_analyse_iter', anonymous=True)

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
base_center_pub = rospy.Publisher(
    "/base_center_pub", PoseStamped, queue_size=10)
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
ell_transformed_msg = tf2_geometry_msgs.do_transform_pose(
    center_pose, transform)

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
ell_transformed_msg = tf2_geometry_msgs.do_transform_pose(
    center_pose, transform)
ell_center_base = np.array([ell_transformed_msg.pose.position.x,
                            ell_transformed_msg.pose.position.y,
                            ell_transformed_msg.pose.position.z])

des_pose_msg = None
rospy.Subscriber("/des_EE_pose", PoseStamped, callback=handle_des_EE_pose)
tmp_pub = rospy.Publisher("/des_EE_pose_tmp", PoseStamped, queue_size=10)

rospy.sleep(0.5)

rospy.loginfo("Waiting for the desired end-effector pose...")
rospy.spin()
