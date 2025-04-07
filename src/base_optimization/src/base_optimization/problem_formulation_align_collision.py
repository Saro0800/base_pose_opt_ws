#!/usr/bin/env python3
import numpy as np
import math
from pymoo.core.problem import ElementwiseProblem
from scipy.spatial.transform import Rotation


class BasePoseOptProblem(ElementwiseProblem):
    def __init__(self, *args):
        # retrieve the passed arguments
        self.ell_center = args[0]
        self.ell_axis_out = args[1]
        self.ell_axis_inn = args[2]
        self.des_pose = args[3]
        self.point_cloud = args[4]

        aO = self.ell_axis_out[0]
        bO = self.ell_axis_out[1]
        
        x_des_EE = self.des_pose[0]
        y_des_EE = self.des_pose[1]
        
        # retrieve the points of the occupancy grid
        self.xcloud = np.array([x[0] for x in self.point_cloud])
        self.ycloud = np.array([x[1] for x in self.point_cloud])
        self.zcloud = np.array([x[2] for x in self.point_cloud])
        
        # des_pose is defined with respect to the fixed frame
        # compute the homog. trans. matrix from R_des to R0
        self.matr_Rdes_R0 = np.zeros((4,4))
        self.matr_Rdes_R0[:3, :3] = Rotation.from_euler('xyz', self.des_pose[3:], degrees=True).as_matrix()
        self.matr_Rdes_R0[:3, 3] = self.des_pose[:3]
        self.matr_Rdes_R0[3, 3] = 1
        
        # compute the coordinate of the versor x with respect to the fixed frame
        self.x_versor_Rdes_omog = np.dot(self.matr_Rdes_R0, np.array([1,0,0,1]))
        # print(self.x_versor_Rdes) CORRETTO
        
        self.des_pose_orig_omog = np.array([self.des_pose[0], self.des_pose[1], self.des_pose[2], 1])
        
        # define the parameters of the optimization problem
        super().__init__(n_var=3,
                         n_obj=1,
                         n_ieq_constr=2,
                         xl=np.array([x_des_EE - max(aO,bO), y_des_EE - max(aO,bO), 0]),
                         xu=np.array([x_des_EE + max(aO,bO), y_des_EE + max(aO,bO), 360]))

    def _evaluate(self, x, out, *args, **kwargs):
        # retrieve the ellipsoid equation equation paramters
        aO = self.ell_axis_out[0]
        bO = self.ell_axis_out[1]
        cO = self.ell_axis_out[2]
        
        aI = self.ell_axis_inn[0]
        bI = self.ell_axis_inn[1]
        cI = self.ell_axis_inn[2]
        
        xc = self.ell_center[0]
        yc = self.ell_center[1]
        zc = self.ell_center[2]

        # retrieve the desired position of the end-effector
        xp = self.des_pose[0]
        yp = self.des_pose[1]
        zp = self.des_pose[2]
        x_ang = self.des_pose[3]
        y_ang = self.des_pose[4]
        z_ang = self.des_pose[5]
                
        # transformation matrix from R0 to Rell
        matr_R0_Rell = np.zeros((4,4))
        rot = np.transpose(Rotation.from_euler('xyz', [0., 0., x[2]], degrees=True).as_matrix())
        matr_R0_Rell[:3, :3] = rot
        matr_R0_Rell[:3, 3] = -np.dot(rot, np.array([x[0], x[1], self.ell_center[2]]))
        matr_R0_Rell[3, 3] = 1
        
        # versor of the x-axis of Rell
        x_versor_Rell_omog = np.array([1,0,0,1])
        
        # compute the coordinate of the origin of the des pose wrt Rell
        orig_des_pose_Rell_omog = np.dot(matr_R0_Rell, self.des_pose_orig_omog)
        
        # compute the coordinate of the versor of Rdes wrt Tell
        xversor_des_pose_Rell_omog = np.dot(matr_R0_Rell, self.x_versor_Rdes_omog)
        
        # compute the projections on the xy plane of Rell
        orig_des_pose_proj_omog = np.array([orig_des_pose_Rell_omog[0],
                                            orig_des_pose_Rell_omog[1],
                                            0,
                                            1])
        xversor_des_pose_Rell_proj_omog = np.array([xversor_des_pose_Rell_omog[0],
                                                     xversor_des_pose_Rell_omog[1],
                                                     0,
                                                     1])
        
        # compute the cos of the agle between the x versor of Rell and the projection
        # of the origin of Rdes
        prod_norm = np.linalg.norm(x_versor_Rell_omog[:3], ord=2) * np.linalg.norm(orig_des_pose_proj_omog[:3], ord=2)
        cos_theta_orig = np.dot(x_versor_Rell_omog[:3], orig_des_pose_proj_omog[:3])/prod_norm
        
        # compute the cos of the agle between the x versor of Rell and the projection
        # of the x versor of Rdes
        prod_norm = np.linalg.norm(x_versor_Rell_omog[:3], ord=2) * np.linalg.norm(xversor_des_pose_Rell_proj_omog[:3], ord=2)
        cos_theta_versor = np.dot(x_versor_Rell_omog[:3], xversor_des_pose_Rell_proj_omog[:3])/prod_norm
        
        # define the angles
        theta_orig = np.rad2deg(np.arccos(cos_theta_orig))
        theta_versor = np.rad2deg(np.arccos(cos_theta_versor))
        
        # count how many points are inside the ellipsoid        
        inner_points = self.point_cloud[((x[0]-self.xcloud)/aO)**2 + ((x[1]-self.ycloud)/bO)**2 + ((zc-self.zcloud)/cO)**2<=1]
        
        # define the constraints
        constrs = [((x[0]-xp)/aO)**2 + ((x[1]-yp)/bO)**2 + ((zc-zp)/cO)**2 - 1,
                   -((x[0]-xp)/aI)**2 - ((x[1]-yp)/bI)**2 - ((zc-zp)/cI)**2 + 1]
        out["G"] = np.row_stack(constrs)
        
        # define the objective function       
        out["F"] = theta_orig + theta_versor + 1.*inner_points.shape[0]
        
        
        
        
        
        
        
        
        
        
        