#
# add path to local library mrob on bashr_rc: "export PYTHONPATH=${PYTHONPATH}:${HOME}/mrob/mrob/lib"
import mrob
import numpy as np
import matplotlib.pyplot as plt

import pickle


# sets the default fonts on matplotlib to type2 postcript
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42


# Exp/Ln chain SO(3)
# ======================================================
# 1) round 0 trajectory. Consistent with manifold error
if 0:
    N, M = 120, 50
    
    eps = np.zeros(N)
    eps[0] = 1e0
    for n in range (1,N):
        eps[n] = eps[n-1]*0.8
    # mean absolute error
    mae = np.zeros(N)
    
    for n in range(N):
        error = []
        for i in range(M):
            xi = np.random.randn(3) 
            xi[0:3] = xi[0:3] / np.linalg.norm(xi[0:3]) * (eps[n]) #around 0
            R = mrob.SO3(xi)
            e = np.linalg.norm(R.R() - mrob.SO3(R.Ln()).R())
            error.append( e )
        mae[n] = sum(error)/M
        print('Iteration n = ', n , ', eps = ', eps[n], ', current error = ', mae[n])
    plt.plot(np.log10(eps),np.log10(mae))
    plt.title('MAE of SO(3) transformation $\epsilon$ to 0')
    #plt.savefig('error_SO3.pdf', bbox_inches='tight')
    # saving data for comparison
    output = open('SO3_zero_norm.pkl', 'wb')
    pickle.dump((mae, eps), output)
    output.close()
    plt.show()

# Exp/Ln chain SO(3)
# ======================================================
# 1) round pi trajectory
if 0:
    N, M = 180, 50
    
    
    eps = np.zeros(N)
    eps[0] = 1e0
    for n in range (1,N):
        eps[n] = eps[n-1]*0.8
    # mean absolute error
    mae = np.zeros(N)
    
    for n in range(N):
        error = []
        for i in range(M):
            xi = np.random.randn(3) 
            xi[0:3] = xi[0:3] / np.linalg.norm(xi[0:3]) * (np.pi - eps[n]) #around pi
            R = mrob.SO3(xi)
            e = np.linalg.norm(R.R() - mrob.SO3(R.Ln()).R())
            error.append( e )
        mae[n] = sum(error)/M
        print('Iteration n = ', n , ', eps = ', eps[n], ', current error = ', mae[n])
    plt.plot(np.log10(eps),np.log10(mae))
    plt.title('MAE of SO(3) transformation $\epsilon$ to $\pi$')
    #plt.savefig('error_SO3.pdf', bbox_inches='tight')
    # saving data for comparison
    output = open('SO3_pi_norm.pkl', 'wb')
    pickle.dump((mae, eps), output)
    output.close()
    plt.show()


# Exp/Ln chain SE(3)
# ======================================================
# 1) round pi trajectory
if 1:
    N, M = 120, 2
    
    
    eps = np.zeros(N)
    eps[0] = 1e0
    for n in range (1,N):
        eps[n] = eps[n-1]*0.8
    # mean absolute error
    mae = np.zeros(N)
    
    for n in range(N):
        error = []
        for i in range(M):
            xi = np.random.randn(3) 
            xi[0:3] = xi[0:3] / np.linalg.norm(xi[0:3]) * (np.pi - eps[n]) #around pi
            T = np.eye(4)
            T[:3,:3] = mrob.SO3(xi).R()
            T[:3,3] = np.random.randn(3)*100;
            print(T)
            T2 = mrob.SE3(mrob.SE3(T).Ln())
            e = np.linalg.norm(T - T2.T())
            print('Diff = \n', T - T2.T())
            error.append( e )
        mae[n] = sum(error)/M
        print('Iteration n = ', n , ', eps = ', eps[n], ', current error = ', mae[n])
    plt.plot(np.log10(eps),np.log10(mae))
    plt.title('MAE of SE(3) transformation $\epsilon$ to $\pi$')
    #plt.savefig('error_SO3.pdf', bbox_inches='tight')
    # saving data for comparison
    output = open('SO3_pi_norm.pkl', 'wb')
    pickle.dump((mae, eps), output)
    output.close()
    plt.show()
