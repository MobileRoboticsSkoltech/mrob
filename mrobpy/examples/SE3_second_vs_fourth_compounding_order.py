import numpy as np
from tqdm import tqdm

import matplotlib
from sys import platform
if platform == "darwin":
    matplotlib.use('PS')
import matplotlib.pyplot as plt

import mrob

from  test_utils import compound_mc

M = 100000
err_1 = []
err_2 = []

alphas = np.linspace(0,1.0,20)#np.arange(0,1.01,0.1)

for alpha in tqdm(alphas):
    xi_1 = np.array([np.pi/6,0,0,0,2,0])
    T_1 = mrob.geometry.SE3(xi_1)
    sigma_1 = alpha*np.diag([1/2,1,1/2,10,5,5])

    cov = mrob.geometry.SE3Cov(T_1, sigma_1)

    xi_2 = np.array([0,np.pi/4,0,0,0,1])
    T_2 = mrob.geometry.SE3(xi_2)
    sigma_2 = alpha*np.diag([1/2,1/2,1,5,10,5])

    T, sigma_mc =  compound_mc(T_1, sigma_1, T_2, sigma_2, M=M)

    cov_2nd = cov.compound_2nd_order(T_2, sigma_2)
    cov_4th = cov.compound_4th_order(T_2, sigma_2)

    err_1.append(np.linalg.norm(sigma_mc - cov_2nd.cov()))
    err_2.append(np.linalg.norm(sigma_mc - cov_4th.cov()))

plt.figure(figsize=(10,10))
plt.title('Error between 2nd and 4th order pose compounding and \nMonte Carlo pose compounding')
plt.plot(alphas,err_1,'-x', label='2nd order')
plt.plot(alphas,err_2,'-x', label='4th order')
plt.xlabel('alpha')
plt.ylabel('error')
plt.grid()
plt.legend()
plt.show()
