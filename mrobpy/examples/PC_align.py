#
# add path to local library mrob on bashr_rc: "export PYTHONPATH=${PYTHONPATH}:${HOME}/mrob/mrob/lib"
import mrob
import numpy as np

# example equal to ./PC_alignment/examples/example_align.cpp
# generate random data
N = 5
X =  np.random.rand(3,N)
T = mrob.SE3(np.random.rand(6))
Y = T.transformArray(X)

print('X = \n', X,'\n T = \n', T.T(),'\n Y =\n', Y)

# solve the problem
T_arun = mrob.ArunSolve(X,Y)
print('Arun solution =\n', T_arun.T())

W = np.ones(N)
T_wp = mrob.WeightedSolve(X,Y,W)
print('Weighted point optimization solution =\n', T_wp.T())
