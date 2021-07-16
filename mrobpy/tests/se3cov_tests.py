import numpy as np
import mrob
import time

def compound_2nd(T_1, sigma_1, T_2, sigma_2):
    T = T_1.mul(T_2)
    T_1_adj = T_1.adj()
    sigma_2_ = T_1_adj@sigma_2@T_1_adj.transpose()
    sigma = sigma_1 + sigma_2_
    return T, sigma

cycles = 1_000_000

start_time = time.time()

xi_1 = np.array([0,0,0,0.5,0,0])
T_1 = mrob.geometry.SE3(xi_1)
sigma_1 = np.diag([0,0,0.01,0.01,0.01,0])

xi_2 = np.array([0,0,1.5,1.0,0,0])
T_2 = mrob.geometry.SE3(xi_2)
sigma_2 = np.diag([0,0,0.1,0.01,0.01,0])

for i in range(cycles):
    compound_2nd(T_1, sigma_1, T_2, sigma_2)

print('pure python:')
time_1 = (time.time() - start_time)
print("%s cycles --- %s seconds ---" % (cycles, time_1))


cov = mrob.geometry.SE3Cov(T_1, sigma_1)

start_time = time.time()

for i in range(cycles):
    cov.compound_2nd_order(T_2, sigma_2)

print('c++ bindings:')
time_2 = (time.time() - start_time)
print("%s cycles --- %s seconds ---" % (cycles, time_2))


print("Pure python / c++ bindings = %s" % (time_1/time_2))