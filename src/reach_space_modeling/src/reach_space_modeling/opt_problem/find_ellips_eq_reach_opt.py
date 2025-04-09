import numpy as np
import rospy
import math
import time


from reach_space_modeling.opt_problem.problem_formulation_reach_opt import EllipsoidEquationOptProblem
from reach_space_modeling.generate_pointcloud.gen_cloud_reach_metric import GenereatePointCloudWithMetric

from pymoo.core.population import Population
from pymoo.algorithms.soo.nonconvex.pattern import PatternSearch
from pymoo.algorithms.soo.nonconvex.ga import GA
from pymoo.algorithms.soo.nonconvex.pso import PSO
from pymoo.optimize import minimize
from pymoo.termination.robust import RobustTermination
from pymoo.termination.ftol import SingleObjectiveSpaceTermination

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from sensor_msgs.point_cloud2 import create_cloud
from reach_space_modeling.srv import ell_params, ell_paramsRequest, ell_paramsResponse
from gazebo_msgs.srv import GetPhysicsProperties

import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


def solve_eqn_prob(points, pnt_weights, alg_name, link=None, center=None, viz_res=False):
    # define the problem
    problem = EllipsoidEquationOptProblem(
        center, points, pnt_weights, viz_res, None, None)

    # define the weights of the points and of the volume, and the optimization algorithm
    problem.num_points_wt = 1
    problem.volume_wt = pow(10, int(math.log10(points.shape[0])-1))

    x0 = np.array(
            [[1, 1, 1, problem.center[0], problem.center[1], problem.center[2]]])

    if alg_name == "GA":
        pop = Population.new(X=x0)
        algorithm = GA(sampling=pop)

    elif alg_name == "PSO":
        init_pop = np.repeat(x0, 25, axis=0)
        algorithm = PSO(sampling=init_pop)

    termination = RobustTermination(
        SingleObjectiveSpaceTermination(tol=pow(10, -6))
    )

    # solve the optimization problem
    res = minimize(problem=problem,
                   algorithm=algorithm,
                   termination=termination,
                   verbose=False,
                   seed=1)

    if viz_res == True:
        print("Best solution found: \n\ta={:.4f}, b={:.4f}, c={:.4f}\n\txC={:.4f}, yC={:.4f}, zC={:.4f}".format(
            res.X[0], res.X[1], res.X[2], res.X[3], res.X[4], res.X[5]))

    print(res.F)

    return res

def create_ell_msg(points, link, res, center=None):
    # retrieve the solution of the opt problem
    a = res.X[0]
    b = res.X[1]
    c = res.X[2]

    # compute the center
    if center is None:
        center = np.array([np.mean([np.min(points[:, 0]), np.max(points[:, 0])]),
                           np.mean([np.min(points[:, 1]),
                                   np.max(points[:, 1])]),
                           np.mean([np.min(points[:, 2]), np.max(points[:, 2])])])

    marker_msg = Marker()
    marker_msg.header.frame_id = link
    marker_msg.header.stamp = rospy.Time.now()

    marker_msg.frame_locked = True

    # set the shape
    marker_msg.type = 2
    marker_msg.id = 0

    # set the scale of the marker
    marker_msg.scale.x = 2*a
    marker_msg.scale.y = 2*b
    marker_msg.scale.z = 2*c

    # set the color
    marker_msg.color.r = 0.0
    marker_msg.color.g = 1.0
    marker_msg.color.b = 0.0
    marker_msg.color.a = 0.4

    # set the pose of the marker
    marker_msg.pose.position.x = center[0]
    marker_msg.pose.position.y = center[1]
    marker_msg.pose.position.z = center[2]
    marker_msg.pose.orientation.x = 0.0
    marker_msg.pose.orientation.y = 0.0
    marker_msg.pose.orientation.z = 0.0
    marker_msg.pose.orientation.w = 1.0

    return marker_msg

def create_cloud_msg(points, link):
    marker_msg = Marker()
    marker_msg.header.frame_id = link
    marker_msg.header.stamp = rospy.Time.now()

    marker_msg.frame_locked = True

    # set the shape
    marker_msg.type = 7
    marker_msg.action = marker_msg.ADD
    marker_msg.id = 0

    # set the scale of the marker
    marker_msg.scale.x = 0.01
    marker_msg.scale.y = 0.01
    marker_msg.scale.z = 0.01

    marker_msg.pose.orientation.x = 0.0
    marker_msg.pose.orientation.y = 0.0
    marker_msg.pose.orientation.z = 0.0
    marker_msg.pose.orientation.w = 1.0

    for p in points:
        # add a point
        marker_msg.points.append(Point(p[0], p[1], p[2]))
        # add a color for the point
        marker_msg.colors.append(ColorRGBA(1.0, 0.0, 0.0, 0.2))

    return marker_msg

def give_ell_params(req):
    a = res.X[0]
    b = res.X[1]
    c = res.X[2]
    xC = res.X[3]
    yC = res.X[4]
    zC = res.X[5]
    ell_ref_frame = link

    return a, b, c, xC, yC, zC, ell_ref_frame


if __name__ == "__main__":
    # get the pointcloud points
    gen_cloud = GenereatePointCloudWithMetric()
    gen_cloud.create_ros_node()

    # # wait for gazebo to be unpaued
    # rospy.wait_for_service("/gazebo/get_physics_properties")

    # get_physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)

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

    # gen_cloud.create_GUI()

    # generate the point cloud
    gen_cloud.from_extern = True
    gen_cloud.urdf_file_path = "~/Desktop/base_pose_opt_ws/src/reach_space_modeling/src/reach_space_modeling/generate_pointcloud/model/mobile_wx250s.urdf"
    gen_cloud.parse_urdf()
    gen_cloud.wrist_lst_j_name = "wrist_rotate"
    gen_cloud.arm_lst_j_name = "elbow"
    gen_cloud.arm_frt_j_name = "waist"
    gen_cloud.num_samples = 20
    gen_cloud.generate_point_cloud()
    rospy.loginfo("Reachability point cloud created...")

    # compute the reachability index for each point
    gen_cloud.generate_reachability_index()
    gen_cloud.vis_cloud_with_measure()

    # count how many points with a given reach measure are available
    freq = np.zeros(int(np.max(gen_cloud.points_reach_measure)+1), dtype=int)
    for i in range(gen_cloud.points_reach_measure.shape[0]):
        freq[int(gen_cloud.points_reach_measure[i])] = freq[int(
            gen_cloud.points_reach_measure[i])] + 1

    for i in range(freq.shape[0]):
        print("Points reachable with {:d} poses: {:d}".format(i, freq[i]))

    # weighted mean of the reachability index
    w_mean = 0
    for i in range(freq.shape[0]):
        w_mean = w_mean + freq[int(i)]*float(i)
    w_mean = w_mean/gen_cloud.points.shape[0]
    print(
        "\nWeighted mean of all reachability measures: {:.2f}\n".format(w_mean))

    # define a weight for each point based on its reachability measure
    pnt_weights = np.zeros(gen_cloud.points.shape[0])
    for i in range(gen_cloud.points.shape[0]):
        pnt_weights[i] = int(w_mean) - gen_cloud.points_reach_measure[i]

    # solve the optimization problem
    points = gen_cloud.points
    link = gen_cloud.point_cloud_orig_frame
    center = np.array([np.mean([np.min(points[:, 0]), np.max(points[:, 0])]),
                       np.mean([np.min(points[:, 1]), np.max(points[:, 1])]),
                       np.mean([np.min(points[:, 2]), np.max(points[:, 2])])])

    start = time.time()

    # alg_name = "GA"
    alg_name = "PSO"
    res = solve_eqn_prob(points, pnt_weights, alg_name,
                         link, center, viz_res=True)

    # figure of a section (xz plane) of the reachability cloud with measure
    tolerance = 0.01
    mask = (gen_cloud.points[:, 1] >= 0 - tolerance) & (gen_cloud.points[:, 1] <= 0 + tolerance)

    sec_points = gen_cloud.points[mask, :]
    # sec_reach_measure = self.points_reach_measure.tolist()
    sec_reach_measure = [gen_cloud.points_reach_measure[i]
                         for i in range(len(mask)) if mask[i]]
    fig2 = plt.figure(figsize=(8, 6))
    ax = fig2.add_subplot()
    sc = ax.scatter(sec_points[:, 0], sec_points[:, 2],
                     c=sec_reach_measure, cmap='plasma_r', s=10)

    # draw an ellipse and its center
    ell = Ellipse((res.X[3], res.X[5]), 2*res.X[0], 2*res.X[2], fill = False)
    ax.add_patch(ell)
    ax.scatter(res.X[3], res.X[5])

    cbar = plt.colorbar(sc)
    plt.xlabel("X")
    plt.ylabel("Z")

    plt.show()

    # end = time.time() - start
    # print("Solution found in {:.4f}s".format(end))

    # center = np.array([res.X[3], res.X[4], res.X[5]])

    # # create the marker msg
    # marker_msg = create_ell_msg(points, link, res, center)

    # # create the center message
    # center = np.expand_dims(center, axis=0)
    # center_msg = create_cloud_msg(center, link)

    # # create the cloud message
    # cloud_msg = create_cloud_msg(points, link)

    # # rospy.init_node('reachability_ellipsoid_publisher', anonymous=True)
    # pub_ellipsoids = rospy.Publisher('/viz_reachability_ellipsoid', Marker, queue_size=10)
    # pub_center = rospy.Publisher('/viz_ellipsoid_center', Marker, queue_size=10)
    # pub_cloud = rospy.Publisher('/viz_pointcloud', Marker, queue_size=10)
    # rate = rospy.Rate(1)  # 1 Hz

    # # provide a service to  get the parameters of the ellipsoid
    # ell_params_srv = rospy.Service("get_ellipsoid_params", ell_params, give_ell_params)

    # print("Start publishing message")
    # for i in range(10):
    #     pub_ellipsoids.publish(marker_msg)
    #     marker_msg.header.stamp = rospy.Time.now()
    #     rate.sleep()

    #     center_msg.header.stamp = rospy.Time.now()
    #     pub_center.publish(center_msg)
    #     rate.sleep()

    #     cloud_msg.header.stamp = rospy.Time.now()
    #     pub_cloud.publish(cloud_msg)
    #     rate.sleep()

    # rospy.spin()
