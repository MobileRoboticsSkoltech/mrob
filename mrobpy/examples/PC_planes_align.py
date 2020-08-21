import mrob
import numpy as np
import open3d

def create_planes(labels, points):
    segments = []
    for l in np.unique(labels):
        if (l != -1):
            segments.append(np.array(points[labels == l]))
    return segments

def plot_segments(segments, color, T = []):
    pcds = []
    for i in segments:
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(i)
        pcd.paint_uniform_color(np.array(color).astype(np.float64))   
        if T!=[]:
	        pcd.transform(T)
        pcds.append(pcd)
    return pcds

def draw_planes_pc(problem):
    pcds = []
    for i in range(problem.get_number_poses()):
        pc = open3d.geometry.PointCloud()
        pc.points = open3d.utility.Vector3dVector(np.array(problem.get_point_cloud(i)))
        s = i/poses
        pc.paint_uniform_color([0.5,1-s, (1-s)/2])
        pc.transform(problem.get_trajectory(i))
        pcds.append(pc)
    open3d.visualization.draw_geometries(pcds)

# 1) Preprecess/Generate points each of them labeled with plane ID
# -----------------------------------------------------------------------------------
points = 500
planes = 3
poses = 3

synthetic_points = mrob.registration.CreatePoints(points,planes,poses, 0.001)
pcds = []
for i in range(poses):
	labels = synthetic_points.get_point_plane_ids(i)
	points = np.array(synthetic_points.get_point_cloud(i))
	s = i/poses
	pcds.extend(plot_segments(create_planes(labels, points), color=[0.5,1-s, (1-s)/2]))
# XXX Plane ids (labels) unused!! for now uniform color with time index
#open3d.visualization.draw_geometries(pcds)


# 2) Generate structure: inputs should be PC with labels and we should get
# -----------------------------------------------------------------------------------
problem = mrob.registration.PlaneRegistration() #empty creator
synthetic_points.create_plane_registration(problem)
#draw_planes_pc(problem)


# 3) Solve Plane aligment linear case
# -----------------------------------------------------------------------------------
problem.solve(mrob.registration.INITIALIZE)
#problem.solve(mrob.registration.GRADIENT)
problem.solve(mrob.registration.GN_HESSIAN)
draw_planes_pc(problem)



# 4) Solve Hessian optimization
problem.reset_solution()
problem.solve(mrob.registration.INITIALIZE)
problem.solve(mrob.registration.LM_HESSIAN)
draw_planes_pc(problem)
r= problem.print_evaluate()
print('overall results([0]error, [1]iters, hessdet[2], conditioningNumber[3]):\n',r)

# printing for hessian at initial steps
#problem.reset_solution()
#problem.solve(mrob.registration.INITIALIZE)
#problem.solve(mrob.registration.GN_CLAMPED_HESSIAN, True)
#r = problem.print_evaluate()
#print('overall results ([0]error, [1]iters, hessdet[2], conditioningNumber[3]):\n',r)

    
if 0:
    for i in range(10):
        problem.solve(mrob.registration.GN_HESSIAN, True)
        draw_planes_pc(problem)
