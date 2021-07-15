
import matplotlib.pyplot as plt
import numpy as np

import mrob

import utils
from  utils import sigma_visualize


# defining two poses with uncertainty to compound
xi_1 = np.array([0,0,0,0.5,0,0])
T_1 = mrob.geometry.SE3(xi_1)
sigma_1 = np.diag([0,0,0.01,0.01,0.01,0])

cov = mrob.geometry.SE3Cov(T_1,sigma_1)

xi_2 = np.array([0,0,1.5,1.0,0,0])
T_2 = mrob.geometry.SE3(xi_2)
sigma_2 = np.diag([0,0,0.1,0.01,0.01,0])

# performing 2nd order compounding of two poses
cov.compound_2nd_order(T_2, sigma_2)
T, sigma = mrob.geometry.SE3(cov.T()), cov.cov()


plt.figure(figsize=(10,10))
plt.title('Second order pose compounding')
sigma_visualize(T_1, sigma_1, label='sigma_1',color='r')
sigma_visualize(T_2, sigma_2, label='sigma_2',color='g')
sigma_visualize(T,sigma, label='2nd order \ncompound',color='b')


sigma_visualize(T,sigma, label='2nd order \ncompound',color='yellow')


plt.grid()
plt.axis('equal')
plt.show()