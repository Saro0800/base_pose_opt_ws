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
        self.des_pose = args[2]

        a = self.ell_axis[0]
        b = self.ell_axis[1]
        
        x_des_EE = self.des_pose[0]
        y_des_EE = self.des_pose[1]
        
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
                         n_ieq_constr=1,
                         xl=np.array([x_des_EE - max(a,b), y_des_EE - max(a,b), 0]),
                         xu=np.array([x_des_EE + max(a,b), y_des_EE + max(a,b), 360]))

    def _evaluate(self, x, out, *args, **kwargs):
        # retrieve the ellipsoid equation equation paramters
        a = self.ell_axis[0]
        b = self.ell_axis[1]
        c = self.ell_axis[2]
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
        
        # define the constraints
        constrs = [((x[0]-xp)/a)**2 + ((x[1]-yp)/b)**2 + ((zc-zp)/c)**2 - 1]
        out["G"] = np.row_stack(constrs)
        
        # define the objective function       
        out["F"] = theta_orig + theta_versor
        
        
        
        
        
        
        
        
        
        
        