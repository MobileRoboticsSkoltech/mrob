# PCRegistration
Point Cloud Registration. Different methods implemented for point cloud registration:
* Arun, and SVD-based method (Arun'1983)
* GICP
* Weighted ICP
* Point-to-point FGraph (solved iteratively)
* Point-to-plane FGraph
* Eigen-factors Fgraph


There are also some routines dedicated to generate sinthetic planes

## Dependencies
C++'14, Eigen


## Files deprecated
estimate_plane.hpp
plane_registration.cpp
plane.hpp