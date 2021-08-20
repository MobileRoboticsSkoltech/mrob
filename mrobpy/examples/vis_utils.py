import numpy as np
import pandas as pd

import mrob

from test_utils import get_mc

from sys import platform

import matplotlib
if platform == "darwin":
    matplotlib.use('PS')

import matplotlib.pyplot as plt

# Here the Cholesky decomposition for singular covariance matrix is implemented
def cholesky(sigma):
    # obtaining map M between original and truncated matrix
    condition =~ (np.all(sigma == 0, axis=1) & (np.all(sigma == 0, axis=0)))
    m = [int(x) for x in condition]
    counter = 0
    res = []
    for el in m:
        if el > 0:
            res.append(counter)
            counter +=1
        else:
            res.append(None)
    M = []    
    for i in range(6):
        tmp = []
        for j in range(6):
            tmp.append([res[i],res[j]])

        M.append(tmp)
    M = np.array(M)

    # obtaining matrix that is obtained by removing zero columns and rows
    block = (sigma[condition,:])[:,condition]

    # applying regular cholesky decomposition
    L = np.linalg.cholesky(block)

    # mapping block decomposition into original matrix
    LL = np.zeros_like(sigma)

    for i in range(LL.shape[0]):
        for j in range(LL.shape[1]):
            if all(M[i,j] != None):
                k = M[i,j][0]
                l = M[i,j][1]
                LL[i,j] = L[k,l]
    # returning resulting factor
    return LL

def get_axis_points(T, sigma, N, K = 1, index = -1, A = None):

    if A is None:
        A = cholesky(sigma)

    points = np.zeros((N,6))
    points[:,index] = np.linspace(-K, K, num = N)

    points_img = (A@points.T).T

    propagated = []

    for i in range(len(points_img)):
        tmp = mrob.geometry.SE3(T)
        tmp.update_lhs(points_img[i])
        propagated.append(tmp)
    pts = np.array([x.t() for x in propagated])
    pts = pts.reshape((-1,3))
    pts = np.vstack((pts,np.array([np.nan,np.nan,np.nan]).reshape(-1,3)))

    return pts

def get_circumference(T,sigma,N,K=1, index_1=-1, index_2=-1, A=None):
    if A is None:
        A = cholesky(sigma)

    points = np.zeros((N,6))
    points[:,index_1] = K*np.cos(np.linspace(0,2*np.pi, num = N))
    points[:,index_2] = K*np.sin(np.linspace(0,2*np.pi, num = N))

    points_img = (A@points.T).T

    propagated = []

    for i in range(len(points_img)):
        tmp = mrob.geometry.SE3(T)
        tmp.update_lhs(points_img[i])
        propagated.append(tmp)
    pts = np.array([x.t() for x in propagated])
    pts = pts.reshape((-1,3))
    pts = np.vstack((pts,np.array([np.nan,np.nan,np.nan]).reshape(-1,3)))

    return pts

def sigma_visualize_3d(T, sigma, N=100, K=1):
    N = 100

    colors = list(matplotlib.colors.CSS4_COLORS.keys())

    A = cholesky(sigma)

    axes = {
        'yaw': get_axis_points(T,sigma,N,K,0,A),
        'pitch': get_axis_points(T,sigma,N,K,1,A),
        'roll': get_axis_points(T,sigma,N,K,2,A),
        'x': get_axis_points(T,sigma,N,K,3,A),
        'y': get_axis_points(T,sigma,N,K,4,A),
        'z': get_axis_points(T,sigma,N,K,5,A)
    }

    circumferences = {
        'yaw vs pitch' : get_circumference(T,sigma,N,K,0,1,A),
        'yaw vs roll' : get_circumference(T,sigma,N,K,0,2,A),
        'yaw vs x' : get_circumference(T,sigma,N,K,0,3,A),
        'yaw vs y' : get_circumference(T,sigma,N,K,0,4,A),
        'yaw vs z' : get_circumference(T,sigma,N,K,0,5,A),

        'pitch vs roll' : get_circumference(T,sigma,N,K,1,2,A),
        'pitch vs x' : get_circumference(T,sigma,N,K,1,3,A),
        'pitch vs y' : get_circumference(T,sigma,N,K,1,4,A),
        'pitch vs z' : get_circumference(T,sigma,N,K,1,5,A),

        'roll vs x' : get_circumference(T,sigma,N,K,2,3,A),
        'roll vs y' : get_circumference(T,sigma,N,K,2,4,A),
        'roll vs z' : get_circumference(T,sigma,N,K,2,5,A),

        'x vs y' : get_circumference(T,sigma,N,K,3,4,A),
        'x vs z' : get_circumference(T,sigma,N,K,3,5,A),

        'y vs z' : get_circumference(T,sigma,N,K,4,5,A),
    }

    return axes, circumferences

def sigma_visualize(T, sigma, N=100, K=[1,1], label="", color=None, ax = None):
    N = 100

    colors = list(matplotlib.colors.CSS4_COLORS.keys())
    
    if color is None:
        color = colors[np.random.randint(0, len(colors))]

    if ax is None:
        ax = matplotlib.pyplot

    ax.plot(T.t()[0], T.t()[1],'x',color=color)
    ax.annotate(label, (T.t()[0], T.t()[1]))
    A = cholesky(sigma)
    for k in set(K):
        # plotting yaw & x plane
        labels = ['+yaw','-yaw','+x','-x']
        points = []
        points.append([0,0,k,0,0,0])
        points.append([0,0,-k,0,0,0])
        points.append([0,0,0,k,0,0])
        points.append([0,0,0,-k,0,0])


        for i in range(N+1):
            points.append([0,0,k*np.cos(2*np.pi/N*i), k*np.sin(2*np.pi/N*i),0,0])

        points = np.array(points)

        points_img = np.dot(A,points.transpose()).transpose()

        propagated = []

        for i in range(len(points_img)):
            tmp = mrob.geometry.SE3(T)
            tmp.update_lhs(points_img[i])
            propagated.append(tmp)
        poses = np.array([x.t() for x in propagated])
        poses = poses.reshape((-1,3))

        ax.plot(poses[4:,0],poses[4:,1], label="{}-sigma yaw & x".format(k), color=color)

        for i in range(len(labels)):
#             ax.annotate(labels[i],xy = (poses[i,0],poses[i,1]), xytext = (poses[i,0]+0.01,poses[i,1]+0.01))
            ax.plot(poses[i,0],poses[i,1],'x',color=color)

        # plotting x & y plane
        labels = ['+x','-x','+y','-y']
        points = []
        points.append([0,0,0,k,0,0])
        points.append([0,0,0,-k,0,0])
        points.append([0,0,0,0,k,0])
        points.append([0,0,0,0,-k,0])


        for i in range(N+1):
            points.append([0,0,0,k*np.cos(2*np.pi/N*i), k*np.sin(2*np.pi/N*i),0])

        points = np.array(points)

        points_img = np.dot(A,points.transpose()).transpose()

        propagated = []

        for i in range(len(points_img)):
            tmp = mrob.geometry.SE3(T)
            tmp.update_lhs(points_img[i])
            propagated.append(tmp)
        poses = np.array([x.t() for x in propagated])
        poses = poses.reshape((-1,3))

        ax.plot(poses[4:,0],poses[4:,1], label="{}-sigma x & y".format(k), color=color)

        for i in range(len(labels)):
#             ax.annotate(labels[i],xy = (poses[i,0],poses[i,1]), xytext = (poses[i,0]+0.01,poses[i,1]+0.01))
            ax.plot(poses[i,0],poses[i,1],'x',color=color)

        # plotting yaw & y plane
        labels = ['+yaw','-yaw','+y','-y']
        points = []
        points.append([0,0,k,0,0,0])
        points.append([0,0,-k,0,0,0])
        points.append([0,0,0,0,k,0])
        points.append([0,0,0,0,k,0])


        for i in range(N+1):
            points.append([0,0,k*np.cos(2*np.pi/N*i),0, k*np.sin(2*np.pi/N*i),0])

        points = np.array(points)

        points_img = np.dot(A,points.transpose()).transpose()

        propagated = []

        for i in range(len(points_img)):
            tmp = mrob.geometry.SE3(T)
            tmp.update_lhs(points_img[i])
            propagated.append(tmp)
        poses = np.array([x.t() for x in propagated])
        poses = poses.reshape((-1,3))

        ax.plot(poses[4:,0],poses[4:,1], label="{}-sigma yaw & y".format(k),color=color)

        for i in range(len(labels)):
#             ax.annotate(labels[i],xy = (poses[i,0],poses[i,1]), xytext = (poses[i,0]+0.01,poses[i,1]+0.01))
            ax.plot(poses[i,0],poses[i,1],'x',color=color)
        # plotting yaw axis of ellipsoid
        points = []

        for i in range(N+1):
            points.append([0,0,k - i*(2*k)/N, 0,0,0])

        points = np.array(points)

        points_img = np.dot(A,points.transpose()).transpose()

        propagated = []

        for i in range(len(points_img)):
            tmp = mrob.geometry.SE3(T)
            tmp.update_lhs(points_img[i])
            propagated.append(tmp)
        poses = np.array([x.t() for x in propagated])
        poses = poses.reshape((-1,3))

        ax.plot(poses[:,0],poses[:,1],color=color)

        # plotting x axis
        points = []
        for i in range(N+1):
            points.append([0,0,0,k - i*(2*k)/N,0,0])

        points = np.array(points)

        points_img = np.dot(A,points.transpose()).transpose()

        propagated = []

        for i in range(len(points_img)):
            tmp = mrob.geometry.SE3(T)
            tmp.update_lhs(points_img[i])
            propagated.append(tmp)
        poses = np.array([x.t() for x in propagated])
        poses = poses.reshape((-1,3))

        ax.plot(poses[:,0],poses[:,1],color=color)

        # plotting y axis
        points = []

        for i in range(N+1):
            points.append([0,0,0,0,k - i*(2*k)/N, 0])

        points = np.array(points)

        points_img = np.dot(A,points.transpose()).transpose()

        propagated = []

        for i in range(len(points_img)):
            tmp = mrob.geometry.SE3(T)
            tmp.update_lhs(points_img[i])
            propagated.append(tmp)
        poses = np.array([x.t() for x in propagated])
        poses = poses.reshape((-1,3))

        ax.plot(poses[:,0],poses[:,1],color=color)


def ellipsoid_wireframe_df(T,sigma,N = 100, K = 1):
    axes, circumferences = sigma_visualize_3d(T=T,sigma=sigma,N = N, K = K)

    df = pd.DataFrame(columns=['x','y','z'])

    for key,val in axes.items():
        tmp = pd.DataFrame(val,columns=['x','y','z'])
        tmp['label'] = key
        df = pd.concat([df,tmp])

    for key,val in circumferences.items():
        tmp = pd.DataFrame(val,columns=['x','y','z'])
        tmp['label'] = key
        df = pd.concat([df,tmp])
    return df

def mc_pointcloud_df(T, sigma, mean=np.zeros(6),N=100):
    poses, xi = get_mc(T=T, sigma=sigma, mean=mean,N=N)

    particles = pd.DataFrame(poses, columns=['x','y','z'])

    return particles
