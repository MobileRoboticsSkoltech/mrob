import mrob
import numpy as np
import open3d




points = 2000
planes = 4
poses = 3

synthetic_points = mrob.registration.CreatePoints(points,planes,poses, 0.05, 0.1) #point noise, bias noise
T_gt = synthetic_points.get_ground_truth_last_pose()
T_gt.print()

# TODO draw points



graph = mrob.FGraph()

ef1 = graph.add_eigen_factor_plane()
print(ef1)
graph.print(True)
