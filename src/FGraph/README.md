# FGraph
Factor Graph library is an implementation to solve the inference problem given by the joint probability P(x,u,z).The solution to this joint probability is equivalent to a Nonlinear Least Squares (NLSQ) problem.
To address this inference problem, we will use a graphical model, Factor Graphs.
Factor Graphs are bipartite graphs, meaning that we express the relations from a set of vertices "nodes" which include our state variables through a set of vertices "factors", capturing the inherent distribution of the nodes variables due to observations.
Bipartite is in the sense that edges of the graph are always from nodes to factors or vice versa. We require two abstract classes,
 * Class Node
 * Class Factor. (see factor.hpp for the conventions on residuals, observations, etc.)
 
 In general, the problem we target is of the form:
 
 C = sum ||f(x) - z||^2_W, where f(x) is a non-linear observation function and z is an observation.
 After linearizontion and staking the output of the sum into a matrix, the problem becomes
 C = 0.5||J dx - (z - f(x)) ||^2 = 0.5||J dx - r ||^2
 
 This library has been designed to interface with python. Please see mrobpy/examples


## Dependencies
C++'14, Eigen



## Coding conventions
Please check the common code conventions for [mrob](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/mrob/browse).

