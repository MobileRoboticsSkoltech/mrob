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
    x = np.zeros((4,3))
    x[0,:] = T.transform(np.array([0,0,0], dtype='float64'))
    x[1,:] = T.transform(np.array([1,0,0], dtype='float64'))
    ax.plot(x[[0,1],0],x[[0,1],1],x[[0,1],2],'r') # X axis
    x[2,:] = T.transform(np.array([0,1,0], dtype='float64'))
    ax.plot(x[[0,2],0],x[[0,2],1],x[[0,2],2],'g') # Y axis
    x[3,:] = T.transform(np.array([0,0,1], dtype='float64'))
    ax.plot(x[[0,3],0],x[[0,3],1],x[[0,3],2],'b') # Z axis
    plt.xlabel('x')
    plt.ylabel('y')
 

# Illustration of different orientations
# --------------------------------------------------------
if 1:
    print('Printing orientations')
    w = np.zeros(3)
    R = mrob.SO3(w)
    R.print()
    ax = plotConfig()
    T = np.eye(4)
    T[:3,:3] = R.R()
    plotT(mrob.SE3(T),ax)# we use this function, but it is equivalent since the translation is set to zero
    plt.title('Rotation Identity')
    plt.savefig('SO3_1.pdf', bbox_inches='tight')  
    plt.show()
    
    
    # Rotation on z axis
    w[2] = -np.pi/4
    R = mrob.SO3(w)
    R.print()
    T[:3,:3] = R.R()
    ax = plotConfig()
    plotT(mrob.SE3(T),ax)
    plt.title('Rotation over the z axis')
    plt.savefig('SO3_2.pdf', bbox_inches='tight')  
    plt.show()
    
    
        
    # Rotation on z axis
    w = np.random.randn(3)
    R = mrob.SO3(w)
    R.print()
    T[:3,:3] = R.R()
    ax = plotConfig()
    plotT(mrob.SE3(T),ax)
    plt.title('Rotation')
    plt.savefig('SO3_3.pdf', bbox_inches='tight')  
    plt.show()
    

# Interpolation TODO: clean code
# --------------------------------------------------------
if 0:
    xi_ini = np.array([0,0,0,0,0,0], dtype='float64')
    #xi_fin = np.array([np.pi/3,1,0,0,0,0], dtype='float64')
    xi_fin = np.random.rand(6)*10
    if np.linalg.norm ( xi_fin[0:3] ) > np.pi:
        xi_fin[0:3] = xi_fin[0:3] / np.linalg.norm ( xi_fin[0:3] ) * (np.pi-1e-5)
    ax = plotConfig()
    N = 20
    xi = np.zeros((N,6))
    t = np.zeros(N)
    for i in range(6):
        xi[:,i] = np.linspace(xi_ini[i],xi_fin[i],N, dtype='float64')
    t = np.linspace(0,1,N, dtype='float64')


    # interpolation in the manifold of se(3)^vee
    # and proper interpolation in SE(3)
    T_0 = mrob.SE3(xi_ini)
    T_0_inv = T_0.inv()
    T_1 = mrob.SE3(xi_fin)
    T_1.print()
    print('direct T1\n',T_1.T())
    mrob.SE3(T_1.T()).print()
    #dxi = mrob.SE3( (T_1.T() @ T_0_inv.T()) ).ln()
    dxi = T_1.mul(T_0_inv).ln()
    for i in range(N):
        Ti = mrob.SE3(xi[i,:])
        plotT(Ti,ax)
        #print(Ti.T())
        
        Ts = mrob.SE3(t[i]*dxi).mul(T_0)
        #print(Ts.T())
        plotT(Ts,ax)
        
        # ploting and visualizing error
        #print(Ts.T())
        print(np.linalg.norm(Ti.ln() - Ts.ln()))
        #T.transform(np.array([0,0,0], dtype='float64'))
    plt.show()

# inverse as the negate in the manifold -xi
if 0:
    e = np.zeros(1000)
    for i in range(1000):
        xi = np.random.rand(6)*3
        Tr = mrob.SE3(xi)
        T = Tr.inv()
        Ti = mrob.SE3(-xi)
        e[i] = np.linalg.norm(T.T()- Ti.T())
        #print( np.linalg.norm(T.T()- Ti.T()))
        
    print(np.max(e), np.min(e))
    
if 0:
    w = np.random.rand(3)
    R = mrob.SO3(w)
    print('SO3 matrix: \n', R.R() )
    print('SO3 in the manifold: \n', R.ln())
    
