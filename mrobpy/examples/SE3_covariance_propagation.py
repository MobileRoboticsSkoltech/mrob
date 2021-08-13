import pandas as pd
import numpy as np
import plotly.express as px
from tqdm import tqdm

import mrob

from vis_utils import sigma_visualize_3d, ellipsoid_wireframe_df, mc_pointcloud_df
from test_utils import get_mc    

def propagate(ps,Rs,sigma0, scene=None):
    
    vis_step = 350
    
    traj = pd.DataFrame(ps,columns=['x','y','z'])
    fig = px.line_3d(traj,x='x',y='y',z='z',height=900)
    
    if scene is not None:
        fig.update_layout(scene=scene)
    
    T = mrob.geometry.SE3(mrob.geometry.SE3(mrob.geometry.SO3(Rs[0]),ps[0]))
    mean = np.zeros(6)
    
    sigma = sigma0
    
    particles = mc_pointcloud_df(T,sigma,mean,N=10000)

    fig.add_scatter3d(x=particles['x'],y=particles['y'],z=particles['z'],opacity=0.5,mode='markers',marker={'size':3})
    
    wframe = ellipsoid_wireframe_df(T,sigma,N=100,K=3)[['x','y','z']]
    fig.add_scatter3d(x=wframe['x'],y = wframe['y'],z=wframe['z'],mode='lines')
    
    cov = mrob.geometry.SE3Cov(T,sigma0)
    for i in tqdm(range(len(ps) - 1)):

        T_next = mrob.geometry.SE3(mrob.geometry.SO3(Rs[i+1]),ps[i+1])
        T_cur = mrob.geometry.SE3(mrob.geometry.SO3(Rs[i]),ps[i])

        tmp = T_cur.inv()*T_next

        cov = cov.compound_2nd_order(tmp, 1e-4*np.diag([0.01,0.01,0.01,0.01,0.01,0.01]))

        if i % vis_step == 0:
            particles =  mc_pointcloud_df(cov.T(),cov.cov(),mean,N=10000)

            fig.add_scatter3d(x=particles['x'],y=particles['y'],z=particles['z'],opacity=0.5,mode='markers',marker={'size':3})

            wframe = ellipsoid_wireframe_df(cov.T(),cov.cov(),N=100,K=3)[['x','y','z']]
            fig.add_scatter3d(x=wframe['x'],y = wframe['y'],z=wframe['z'],mode='lines')
            
    fig.update_yaxes(scaleanchor = "x", scaleratio = 1)
    fig.show()

class IMU:
    def __init__(self, start_R, start_v, start_p, g=9.81):
        self.R = start_R
        self.v = start_v
        self.p = start_p
        # self.g = np.array([0, 0, g])
        self.R_history, self.v_history, self.p_history = [], [], []

    def integrate(self, omega, a, dt):
        rotation = mrob.geometry.SO3(omega * dt)
        R = self.R.mul(rotation)
        v = self.v + self.R.R() @ (a * dt) # + self.g * dt 
        p = self.p + self.v * dt + 1/2 * self.R.R() @ (a * (dt ** 2)) #  1/2 * self.g * (dt ** 2)
        self.R = R 
        self.v = v 
        self.p = p
        self.R_history.append(R)
        self.v_history.append(v)
        self.p_history.append(p)

        return R, v, p

    def get_history(self):
        return self.R_history, self.v_history, self.p_history


def propagate_on_trajectory(data_path, start_idx, finish_idx, sigma_0, scene):
    # reading the data
    data = pd.read_csv(data_path)
    dt = np.ones(data.shape[0]) * 0.005
    angle_v = np.array(data[['omega_x', 'omega_y', 'omega_z']])
    angle_a = np.array(data[['acc_x', 'acc_y', 'acc_z']])

    # integrating the data
    imu = IMU(mrob.geometry.SO3(np.eye(3)), np.array(data[['omega_x', 'omega_y', 'omega_z']].iloc[0]), np.array(data[['x', 'y', 'z']].iloc[0]))
    for i in range(angle_v.shape[0]):
        imu.integrate(angle_v[i], angle_a[i], dt[i])

    # copy resulting pose and orientation history 
    Rs = [R.R() for R in imu.R_history]
    ps = imu.p_history = np.array(imu.p_history)

    # propagating along part of the circular trajectory
    propagate(ps[start_idx:finish_idx],Rs[start_idx:finish_idx],sigma_0, scene = scene)

if __name__ == "__main__":


    # initial covariance
    sigma_0 = np.diag([0.0,0.0,0.0,0.01,0.01,0.01])


    # EXAMPLE #1: ALONG CIRCLE
    # defining parameters of plotly scene
    scene = dict(aspectmode='manual', aspectratio=dict(x=1, y=1, z=1),
                                    xaxis = dict(nticks=4, range=[-15,15],),
                                    yaxis = dict(nticks=4, range=[-15,15],),
                                    zaxis = dict(nticks=4, range=[-15,15],))

    propagate_on_trajectory("./mrobpy/examples/data/acceleration_along_circle.csv",
                                start_idx=2000,
                                finish_idx=7000,
                                sigma_0 = sigma_0,
                                scene = scene)


    # EXAMPLE #2: ALONG LINE
    propagate_on_trajectory("./mrobpy/examples/data/acceleration_along_straigt_line.csv",
                                start_idx=0,
                                finish_idx=-1,
                                sigma_0 = sigma_0,
                                scene = None)

    