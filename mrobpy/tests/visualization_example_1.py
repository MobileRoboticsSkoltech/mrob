import matplotlib.pyplot as plt
import numpy as np

import mrob

import utils
from  utils import sigma_visualize, get_mc

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