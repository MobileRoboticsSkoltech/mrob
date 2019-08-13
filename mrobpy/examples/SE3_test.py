#
# add path to local library mrob on bashr_rc: "export PYTHONPATH=${PYTHONPATH}:${HOME}/mrob/mrob/lib"
import mrob
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# sets the default fonts on matplotlib to type2 postcript
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42


def plotConfig():
    "configfures the 3d plot structure for representing tranformations"
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    return ax


def plotT(T, ax):
    "Plots a 3 axis frame in the origin given the mrob SE3 transformation, right-hand convention"
    # transform 3 axis to the coordinate system
    x = np.zeros((4, 3))
    x[0, :] = T.transform(np.array([0, 0, 0], dtype='float64'))
    x[1, :] = T.transform(np.array([1, 0, 0], dtype='float64'))
    ax.plot(x[[0, 1], 0], x[[0, 1], 1], x[[0, 1], 2], 'r')  # X axis
    x[2, :] = T.transform(np.array([0, 1, 0], dtype='float64'))
    ax.plot(x[[0, 2], 0], x[[0, 2], 1], x[[0, 2], 2], 'g')  # Y axis
    x[3, :] = T.transform(np.array([0, 0, 1], dtype='float64'))
    ax.plot(x[[0, 3], 0], x[[0, 3], 1], x[[0, 3], 2], 'b')  # Z axis


# Qualitative test for some issues found while creating SE3 in mrob
# 1) round pi trajectory
if 0:
    xi_ini = np.array([0, 0, 0, 0, 0, 0], dtype='float64')
    # xi_fin = np.array([np.pi/3,1,0,0,0,0], dtype='float64')
    xi_fin = np.random.rand(6) * 10
    if np.linalg.norm(xi_fin[0:3]) > np.pi:
        xi_fin[0:3] = xi_fin[0:3] / np.linalg.norm(xi_fin[0:3]) * (np.pi - 0.1)
    ax = plotConfig()
    N = 20
    xi = np.zeros((N, 6))
    t = np.zeros(N)
    for i in range(6):
        xi[:, i] = np.linspace(xi_ini[i], xi_fin[i], N, dtype='float64')
    t = np.linspace(0, 1, N, dtype='float64')
    
    #plt.savefig('name.pdf', bbox_inches='tight')


# 2) Life of objects, some data gets corrupted when chaining functions/operators
if 1:
    xi = np.random.rand(6)
    T1 = mrob.SE3(xi)
    T2 = mrob.SE3(np.random.rand(6))
    print(T1.T(), '\n and directly does not work, T = \n', mrob.SE3(xi).inv().T() , '\n is SE3? ', mrob.isSE3(mrob.SE3(xi).inv().T()))
    
    T3 = mrob.SE3(T1.T() @ T2.inv().T())
    T3.print()
    T4 = T1.mul(T2.inv()).mul(T3)
    T4.print()
    print ('is numpy muls a SE3? ', mrob.isSE3(T3.T()), '\n is mul a SE3?', mrob.isSE3(T4.T()))
