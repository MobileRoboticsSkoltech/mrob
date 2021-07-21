import matplotlib.pyplot as plt
import numpy as np

import mrob

import utils
from  utils import sigma_visualize, compound_mc, get_mc



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