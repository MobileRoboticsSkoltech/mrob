import matplotlib.pyplot as plt
import numpy as np
import plotly.express as px
import pandas as pd


import mrob

# import utils
from  vis_utils import sigma_visualize, sigma_visualize_3d
from test_utils import get_mc, compound_mc

# EXAMPLE #1
#================================================================
plt.figure(figsize=(10,10))
plt.suptitle('Covariance ellipsoid visualisation v.s. \ncorresponding Monte Carlo direct sampling distribution')
sigma = np.diag([0,0,0.1,0.001,0.01,0])
mean = np.array([0,0,0,0,0,0])
poses, xi = get_mc(mrob.geometry.SE3([0,0,0,1,0,0]), sigma, mean,N=1_000_0)
sigma_visualize(mrob.geometry.SE3([0,0,0,1,0,0]), sigma=sigma,N = 100, K=[3], color='red')
plt.scatter(poses[:,0],poses[:,1], label='Monte Carlo',s=2)
plt.xlim([-1.5,1.5])
plt.ylim([-1.5,1.5])
plt.grid()

plt.xlabel('X')
plt.ylabel('Y')
plt.show()

#EXAMPLE #2
#================================================================
# defining two poses with uncertainty to compound
xi_1 = np.array([0,0,0,0.5,0,0])
T_1 = mrob.geometry.SE3(xi_1)
sigma_1 = np.diag([0,0,0.01,0.01,0.01,0])

xi_2 = np.array([0,0,1.5,1.0,0,0])
T_2 = mrob.geometry.SE3(xi_2)
sigma_2 = np.diag([0,0,0.1,0.01,0.01,0])

# performing monte carlo compounding of two poses
T,sigma = compound_mc(T_1, sigma_1, T_2, sigma_2, M=100_000)

plt.figure(figsize=(10,10))
plt.title('Monte Carlo pose compounding')
sigma_visualize(T_1, sigma_1, label='sigma_1',color='r')
sigma_visualize(T_2, sigma_2, label='sigma_2',color='g')
sigma_visualize(T,sigma,label='compound',color='b')

poses, xi  = get_mc(T, sigma,N=20000)

plt.scatter(poses[:,0],poses[:,1], label='Monte Carlo',s = 2)
plt.grid()
plt.axis('equal')
plt.show()


# EXAMPLE #3
#================================================================
# defining two poses with uncertainty to compound
xi_1 = np.array([0,0,0,0.5,0,0])
T_1 = mrob.geometry.SE3(xi_1)
sigma_1 = np.diag([0,0,0.01,0.01,0.01,0])

cov = mrob.geometry.SE3Cov(T_1,sigma_1)

xi_2 = np.array([0,0,1.5,1.0,0,0])
T_2 = mrob.geometry.SE3(xi_2)
sigma_2 = np.diag([0,0,0.1,0.01,0.01,0])

# performing 2nd order compounding of two poses
new_cov = cov.compound_2nd_order(T_2, sigma_2)
T, sigma = mrob.geometry.SE3(new_cov.T()), new_cov.cov()

plt.figure(figsize=(10,10))
plt.title('Second order pose compounding')
sigma_visualize(T_1, sigma_1, label='sigma_1',color='r')
sigma_visualize(T_2, sigma_2, label='sigma_2',color='g')
sigma_visualize(T,sigma, label='2nd order \ncompound',color='b')

plt.grid()
plt.axis('equal')
plt.show()

# EXAMPLE #4
#================================================================
plt.figure(figsize=(10,10))
plt.suptitle('Covariance ellipsoid visualisation v.s. \ncorresponding Monte Carlo direct sampling distribution')
# sigma = np.diag([.3,.01,0.01,0.01,0.01,0.02])
mean = np.array([0,0,0,0,0,0])

T = mrob.geometry.SE3([0.1,-0.1,0.05,1,0.05,0.-1])
sigma = np.diag([0.01,0.01,0.1,0.001,0.001,0.001])

axes, circumferences = sigma_visualize_3d(T,sigma,N = 100,K = 3)

df = pd.DataFrame(columns = ['x','y','z'])

for key,val in axes.items():
    tmp = pd.DataFrame(val,columns=['x','y','z'])
    tmp['label'] = key
    df = pd.concat([df,tmp])

for key,val in circumferences.items():
    tmp = pd.DataFrame(val,columns=['x','y','z'])
    tmp['label'] = key
    df = pd.concat([df,tmp])

fig = px.line_3d(data_frame=df,x='x',y='y',z='z',color='label',hover_name='label')

# Noise has zero mean values
mean = np.zeros(6)
poses, xi = get_mc(T, sigma, mean,N=1_000)
particles = pd.DataFrame(poses, columns=['x','y','z'])
fig.add_scatter3d(x=particles['x'],y=particles['y'],z=particles['z'],opacity=0.5,mode='markers',marker=dict(size=3))
fig.show()
