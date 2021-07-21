import matplotlib.pyplot as plt
import numpy as np
from numpy.core.fromnumeric import size
import pandas as pd

import mrob

import utils
from  utils import sigma_visualize_3d, get_mc

import plotly.express as px

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
# fig.update_layout(scene = dict(
#         xaxis = dict(nticks=4, range=[0.5,1.5],),
#                      yaxis = dict(nticks=4, range=[-0.5,0.5],),
#                      zaxis = dict(nticks=4, range=[-0.5,0.5],),))



# Noise has zero mean values
mean = np.zeros(6)

# Noise covariance matrix
# sigma = np.diag([0,0,0.1,0.001,0.01,0])

poses, xi = get_mc(T, sigma, mean,N=1_000)

particles = pd.DataFrame(poses, columns=['x','y','z'])

fig.add_scatter3d(x=particles['x'],y=particles['y'],z=particles['z'],opacity=0.5,mode='markers',marker=dict(size=3))


fig.show()