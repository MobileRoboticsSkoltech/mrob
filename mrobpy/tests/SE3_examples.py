# This test is a composition of various problems involving SE3.
# TODO add more cases of use, now it is an imcomplete test
# add path to local library mrob on bashr_rc: "export PYTHONPATH=${PYTHONPATH}:${HOME}/mrob/mrob/lib"
import mrob
import numpy as np

# Illustration of different orientations
# --------------------------------------------------------

print('Printing orientations')
w = np.zeros(3)
R = mrob.geometry.SO3(w)
R.print()
T = np.eye(4)
T[:3,:3] = R.R()


# Rotation on z axis
w[2] = -np.pi/4
R = mrob.geometry.SO3(w)
R.print()
T[:3,:3] = R.R()

    
# Rotation on z axis
w = np.random.randn(3)
R = mrob.geometry.SO3(w)
R.print()
T[:3,:3] = R.R()

# Interpolation 
# --------------------------------------------------------
xi_ini = np.array([0,0,0,0,0,0], dtype='float64')
#xi_fin = np.array([np.pi/3,1,0,0,0,0], dtype='float64')
xi_fin = np.random.rand(6)*10
if np.linalg.norm ( xi_fin[0:3] ) > np.pi:
    xi_fin[0:3] = xi_fin[0:3] / np.linalg.norm ( xi_fin[0:3] ) * (np.pi-1e-5)
N = 20
xi = np.zeros((N,6))
t = np.zeros(N)
for i in range(6):
    xi[:,i] = np.linspace(xi_ini[i],xi_fin[i],N, dtype='float64')
t = np.linspace(0,1,N, dtype='float64')


# interpolation in the manifold of se(3)^vee
# and proper interpolation in SE(3)
T_0 = mrob.geometry.SE3(xi_ini)
T_0_inv = T_0.inv()
T_1 = mrob.geometry.SE3(xi_fin)
T_1.print()
print('direct T1\n',T_1.T())
mrob.geometry.SE3(T_1.T()).print()
#dxi = mrob.geometry.SE3( (T_1.T() @ T_0_inv.T()) ).Ln()
dxi = T_1.mul(T_0_inv).Ln()
for i in range(N):
    Ti = mrob.geometry.SE3(xi[i,:])
    print(Ti.T())
    
    Ts = mrob.geometry.SE3(t[i]*dxi).mul(T_0)
    print(Ts.T())
    
    # ploting and visualizing error
    #print(Ts.T())
    print(np.linalg.norm(Ti.Ln() - Ts.Ln()))
    #T.transform(np.array([0,0,0], dtype='float64'))

# inverse as the negate in the manifold -xi
e = np.zeros(1000)
for i in range(1000):
    xi = np.random.rand(6)*3
    Tr = mrob.geometry.SE3(xi)
    T = Tr.inv()
    Ti = mrob.geometry.SE3(-xi)
    e[i] = np.linalg.norm(T.T()- Ti.T())
    #print( np.linalg.norm(T.T()- Ti.T()))
    
print(np.max(e), np.min(e))
    
w = np.random.rand(3)
R = mrob.geometry.SO3(w)
print('SO3 matrix: \n', R.R() )
print('SO3 in the manifold: \n', R.Ln())
    
