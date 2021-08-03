#
# add path to local library mrob on bashr_rc: "export PYTHONPATH=${PYTHONPATH}:${HOME}/mrob/mrob/lib"
import mrob
import numpy as np
import open3d
# example equal to ./PC_alignment/examples/example_align.cpp
# generate random data
N = 500
X =  np.random.rand(N,3)
T = mrob.geometry.SE3(np.random.rand(6))
Y = T.transform_array(X)

print('X = \n', X,'\n T = \n', T.T(),'\n Y =\n', Y)




def pcd_1(X, color, T = np.identity(4)):
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(X)
    pcd.transform(T)
    pcd.paint_uniform_color(color)
    return pcd

def vis_her(X, Y, T = np.identity(4)):
    blue = np.array([0,0,1], dtype='float64')
    red = np.array([1,0,0], dtype='float64')
    open3d.visualization.draw_geometries([pcd_1(X,red), pcd_1(Y,blue, T)])

def vis_arr(X):
	pcd = open3d.PointCloud()
	pcd.points =open3d.utility.Vector3dVector(X)
	pcd.paint_uniform_color(np.random.rand(3,).astype(np.float64))
	open3d.visualization.draw_geometries([pcd])



# solve the problem
vis_her(X,Y)
T_arun = mrob.registration.arun(X,Y)
print('Arun solution =\n', T_arun.T())
vis_her(Y,X,np.asarray(T_arun.T()))

W = np.ones(N)
T_wp = mrob.registration.weighted(X,Y,W)
print('Weighted point optimization solution =\n', T_wp.T())
