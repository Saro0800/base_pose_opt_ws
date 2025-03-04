import tkinter as tk
import numpy as np
from scipy.spatial.transform import Rotation
from reach_space_modeling.generate_pointcloud.gen_cloud_GUI import GenereatePointCloud
import pytorch_kinematics as pk
import torch
import time

import tkinter as tk
import numpy as np
import time
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class GenereatePointCloudSingularConf(GenereatePointCloud):   
    def __init__(self) -> None:
        super().__init__()

    def generate_point_cloud(self):
        # check if the last wrist joint is selected
        if self.from_extern == False:
            self.wrist_lst_j_name = self.wrist_lst_j.get()
            if self.wrist_lst_j_name==None or self.wrist_lst_j_name=="":
                self.text_box.config(state='normal')
                self.text_box.insert('end', "The last joint of the wrist has not been selected\n",'error')
                self.text_box.yview(tk.END)
                self.text_box.config(state='disabled')
                return

            # check if the last arm joint is selected
            self.arm_lst_j_name = self.arm_lst_j.get()
            if self.arm_lst_j_name==None or self.arm_lst_j_name=="":
                self.text_box.config(state='normal')
                self.text_box.insert('end', "The last joint of the arm has not been selected\n",'error')
                self.text_box.yview(tk.END)
                self.text_box.config(state='disabled')
                return
            
            # check if the first arm joint is selected
            self.arm_frt_j_name = self.arm_frt_j.get()
            if self.arm_frt_j_name==None or self.arm_frt_j_name=="":
                self.text_box.config(state='normal')
                self.text_box.insert('end', "The first joint of the arm has not been selected\n",'error')
                self.text_box.yview(tk.END)
                self.text_box.config(state='disabled')
                return
            
            # check if the numbe rof samples per joint has been entered
            self.num_samples = self.num_samples_spinbox.get()
            if self.num_samples==None or self.num_samples=="":
                self.text_box.config(state='normal')
                self.text_box.insert('end', "The number of samples per joint has not been enetered\n",'error')
                self.text_box.yview(tk.END)
                self.text_box.config(state='disabled')
                return
            self.num_samples = int(self.num_samples)
        
        # retrieve the last wrist joint
        wrist_lst_joint = self.robot.joint_map.get(self.wrist_lst_j_name)

        # let the user select the last arm joint
        arm_lst_joint = self.robot.joint_map.get(self.arm_lst_j_name)
        
        # let the user select the first arm joint
        arm_frt_joint = self.robot.joint_map.get(self.arm_frt_j_name)
        
        # create a dictionary to retrieve the name of the joint from the parent or child link name
        plink_2_joint_name = {}
        clink_2_joint_name = {}
        for joint_name, joint in self.robot.joint_map.items():
            plink_2_joint_name[joint.parent] = joint_name
            clink_2_joint_name[joint.child] = joint_name

        # start the timer
        start = time.time()

        # compute the coordinates of the representative point wrt to last arm joint
        curr_joint = wrist_lst_joint
        if curr_joint!=arm_lst_joint:
            rpp_coords = curr_joint.origin[:,3]
        else:
            rpp_coords = np.array([0, 0, 0, 1])

        while(curr_joint!=arm_lst_joint and curr_joint.parent != arm_lst_joint.child):
            prec_joint = self.robot.joint_map.get(clink_2_joint_name[curr_joint.parent])
            T0 = prec_joint.origin
            rpp_coords = np.dot(T0,np.transpose(rpp_coords))
            curr_joint = prec_joint

        points = np.array(rpp_coords)
        points = np.expand_dims(points,1)

        # compute all the possible positions of the representative points
        curr_joint = arm_lst_joint
        self.q_val = []
        while(True):
            T0 = curr_joint.origin
            # create a span of values for the desired joint
            steps = np.linspace(curr_joint.limit.lower, curr_joint.limit.upper, self.num_samples)
            
            if self.from_extern == False:
                self.text_box.config(state='normal')
                self.text_box.insert('end', "Computing points resulting from rotation of {}... ".format(curr_joint.name))
                self.text_box.yview(tk.END)
                self.text_box.config(state='disabled')

            new_points = []

            # multiply all previous points for the matrix 
            for step in steps:
                T1= np.zeros((4,4))
                T1[3,3] = 1

                if curr_joint.axis[0]==1:
                    T1[:3,:3] = Rotation.from_euler('x',step).as_matrix()
                elif curr_joint.axis[1]==1:
                    T1[:3,:3] = Rotation.from_euler('y',step).as_matrix()
                elif curr_joint.axis[2]==1:
                    T1[:3,:3] = Rotation.from_euler('z',step).as_matrix()

                tmp = np.dot(T1, points)
                tmp = np.dot(T0, tmp)
                new_points.append(tmp)
            
            if len(self.q_val)==0:
                for step in steps:
                    self.q_val.append([step])
            else:
                q_val_new = []
                for step in steps:
                    for q in self.q_val:
                        q_val_new.append(np.insert(q,0,step))
                self.q_val = q_val_new

            
            
            if self.from_extern == False:
                self.text_box.config(state='normal')
                self.text_box.insert('end', "done\n")
                self.text_box.yview(tk.END)
                self.text_box.config(state='disabled')

            new_points = np.array(new_points)
            new_points = np.concatenate(new_points, axis=1)
            points = new_points

            if curr_joint == arm_frt_joint:
                break

            if curr_joint.parent in clink_2_joint_name:
                curr_joint = self.robot.joint_map.get(clink_2_joint_name[curr_joint.parent])
            else:
                break

        self.gen_time = time.time()-start

        if self.from_extern == False:
            self.text_box.config(state="normal")
            self.text_box.insert('end', "Point cloud generated in: {}\n".format(self.gen_time))
            self.text_box.yview(tk.END)
            self.text_box.config(state='disabled')

            self.text_box.config(state='normal')
            self.text_box.insert('end', "\nAll arm's joints considered. Total numer of points: {}\n".format(points.shape[1]))
            self.text_box.yview(tk.END)
            self.text_box.config(state='disabled')
        
        self.points = points[:3,:].transpose()
        self.point_cloud_orig_frame = curr_joint.parent

        # activate the publish button
        if self.from_extern == False:
            self.pub_msg.config(state='active')
  
    def generate_reachability_index(self):
        # create a pk model of the robot from the urdf file
        robot_chain = pk.build_chain_from_urdf(open(self.urdf_file_path).read())
        robot_chain.print_tree()
        
        # extract the manipulator serial chain
        orig = "locobot/arm_base_link"
        end = "locobot/gripper_link"
        manipulator_chain = pk.SerialChain(robot_chain, end, orig)
        
        # score list for each reachable point
        self.points_reach_measure = []

        # for each config of the arm's joints, compute the jacobian matrix
        for q in self.q_val:
            full_q = np.concatenate((q, np.pi/3*0*np.ones(3)))
            th = torch.tensor(full_q, dtype=torch.float32)
            jac = np.array(manipulator_chain.jacobian(th)).reshape(6,6)
            index = np.abs(np.linalg.det(jac))
            self.points_reach_measure.append(index)
        
    def vis_cloud_with_measure(self):
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')

        color_values = (self.points_reach_measure - np.min(self.points_reach_measure)) / (np.max(self.points_reach_measure) - np.min(self.points_reach_measure))
        sc = ax.scatter(self.points[:, 0], self.points[:, 1], self.points[:, 2], 
                        c=self.points_reach_measure, cmap='plasma_r', s=10)

        cbar = plt.colorbar(sc, ax=ax, shrink=0.5)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        tolerance = 0.01
        mask = (self.points[:,1]>=0-tolerance) & (self.points[:,1]<=0+tolerance)
        # print(mask)
        # print(mask.shape)
        # print(type(self.points_reach_measure))
        # print(type(self.points))
        
        sec_points = self.points[mask,:]
        # sec_reach_measure = self.points_reach_measure.tolist()
        sec_reach_measure = [self.points_reach_measure[i] for i in range(len(mask)) if mask[i]]
        fig2 = plt.figure(figsize=(8, 6))
        sc = plt.scatter(sec_points[:,0], sec_points[:,2], c=sec_reach_measure, cmap='plasma_r', s=10)
        
        cbar = plt.colorbar(sc)
        plt.xlabel("X")
        plt.ylabel("Z")
        
        plt.show()
        
        
        
        

if __name__=="__main__":
    gen_cloud = GenereatePointCloudSingularConf()
    gen_cloud.create_ros_node()
    gen_cloud.create_GUI()
    gen_cloud.generate_reachability_index()
    gen_cloud.vis_cloud_with_measure()
