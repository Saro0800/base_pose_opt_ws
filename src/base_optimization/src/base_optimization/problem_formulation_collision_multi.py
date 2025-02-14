#!/usr/bin/env python3
import numpy as np
import math
from pymoo.core.problem import ElementwiseProblem
from scipy.spatial.transform import Rotation


class BasePoseOptProblem(ElementwiseProblem):
    def __init__(self, *args):
        # retrieve the passed arguments
        self.ell_center = args[0]
        self.ell_axis = args[1]
        self.des_pose_multi = args[2]
        self.point_cloud = args[3]

        a = self.ell_axis[0]
        b = self.ell_axis[1]

        # retrieve the points of the occupancy grid
        self.xcloud = np.array([x[0] for x in self.point_cloud])
        self.ycloud = np.array([x[1] for x in self.point_cloud])
        self.zcloud = np.array([x[2] for x in self.point_cloud])

        # define the parameters of the optimization problem
        super().__init__(n_var=3,
                         n_obj=1,
                         n_ieq_constr=3,
                         xl=np.array([min([p.x for p in self.des_pose_multi]) - max(a, b),
                                      min([p.y for p in self.des_pose_multi]
                                          ) - max(a, b),
                                      0]),
                         xu=np.array([max([p.x for p in self.des_pose_multi]) + max(a, b),
                                      max([p.y for p in self.des_pose_multi]
                                          ) + max(a, b),
                                      360]))

    def _evaluate(self, x, out, *args, **kwargs):
        # retrieve the ellipsoid equation equation paramters
        a = self.ell_axis[0]
        b = self.ell_axis[1]
        c = self.ell_axis[2]
        xc = self.ell_center[0]
        yc = self.ell_center[1]
        zc = self.ell_center[2]

        # transformation matrix from R0 to Rell
        matr_R0_Rell = np.zeros((4, 4))
        rot = np.transpose(Rotation.from_euler(
            'xyz', [0., 0., x[2]], degrees=True).as_matrix())
        matr_R0_Rell[:3, :3] = rot
        matr_R0_Rell[:3, 3] = -np.dot(rot, np.array([x[0], x[1], self.ell_center[2]]))
        matr_R0_Rell[3, 3] = 1

        # versor of the x-axis of Rell
        x_versor_Rell_omog = np.array([1, 0, 0, 1])

        # compute the angle between each vector representing the position
        # of the des point wrt to Rell, and the xversor of Rell
        angles_multi = []
        sin_multi = []
        n_sx = 0
        n_dx = 0
        x_multi = []
        
        # compute how many des poses are inside the ellipsoid
        pose_reached_multi = 0
        pose_reached_multi_inner = 0
        
        for p in self.des_pose_multi:
            if ((x[0]-p.x)/a)**2 + ((x[1]-p.y)/b)**2 + ((zc-p.z)/c)**2 <= 1:
                pose_reached_multi = pose_reached_multi + 1

                # compute the homog coordinates of each des point wrt to Rell
                p_R0 = np.array([p.x, p.y, p.z, 1])
                p_Rell = np.dot(matr_R0_Rell, p_R0)
                # prod_norms = np.linalg.norm(x_versor_Rell_omog[:3], ord=2) * np.linalg.norm(p_Rell[:3], ord=2)
                # # compute the cosin of the angles between the 2 vectors, then the angle
                # cos_angle = np.dot(x_versor_Rell_omog[:3], p_Rell[:3])/prod_norms
                # angle = np.rad2deg(np.arccos(cos_angle))
                # # angle = np.abs(angle - 180)
                angle = np.arctan2(p_Rell[1], p_Rell[0])
                angles_multi.append(angle)
                sin_cat = np.sin(angle)*np.linalg.norm(p_Rell, ord=2)
                
                if p_Rell[1]>=0:
                    n_sx = n_sx + 1 + sin_cat
                else:
                    n_dx = n_dx + 1 + sin_cat
                    # sin_cat = -sin_cat
                    
                sin_multi.append(sin_cat)
                x_multi.append(p_Rell[0])
                
            # compute how many des poses are inside the ellipsoid
            if ((x[0]-p.x)/(0.3*a))**2 + ((x[1]-p.y)/(0.3*b))**2 + ((zc-p.z)/(0.3*c))**2 <= 1:
                pose_reached_multi = pose_reached_multi - 1
                pose_reached_multi_inner = pose_reached_multi_inner + 1
                
        

        # count how many points are inside the ellipsoid (collisions)
        inner_points = self.point_cloud[(
            (x[0]-self.xcloud)/a)**2 + ((x[1]-self.ycloud)/b)**2 + ((zc-self.zcloud)/c)**2 <= 1]

        # # define the constraints
        # constrs = [((x[0]-xp)/a)**2 + ((x[1]-yp)/b)**2 + ((zc-zp)/c)**2 - 1]
        constrs = [-pose_reached_multi + 1,
                   - np.sum(x_multi),
                   pose_reached_multi_inner
                ]
        out["G"] = np.row_stack(constrs)

        # define the objective function
        # print(pose_reached_multi)
        alpha = 1
        beta = 10
        gamma = 1
        
        # print(inner_points.shape[0])
        # print(pose_reached_multi)
        # print(np.sum(sin_multi))
        
        # out["F"] = alpha*inner_points.shape[0] - beta*pose_reached_multi + gamma*np.abs(np.sum(angles_multi))
        # out["F"] = alpha*inner_points.shape[0] - beta*pose_reached_multi + gamma*np.abs(np.sum(sin_multi))
        # out["F"] = alpha*inner_points.shape[0] - beta*pose_reached_multi + gamma*np.abs(np.sum(sin_multi))
        out["F"] =  alpha*inner_points.shape[0]\
                    - beta*pose_reached_multi\
                    + np.abs(np.sum(angles_multi))\
                    + np.abs(n_sx - n_dx)\
                    # - np.sum(x_multi)
