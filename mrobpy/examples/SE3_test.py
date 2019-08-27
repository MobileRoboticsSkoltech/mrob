#
# add path to local library mrob on bashr_rc: "export PYTHONPATH=${PYTHONPATH}:${HOME}/mrob/mrob/lib"
import mrob
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import pickle


# sets the default fonts on matplotlib to type2 postcript
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42


# Overall error for differente segments of theta (angle)
if 1:
    print('Overall evaluation of SO(3) ranging from theta [0,pi]')
    N, M = 200, 100
    error = np.zeros((N,M))
    rmse = np.zeros(N)
    eps = 1e-5 #to avoid the explicit case 0,pi which gives an exact solution, but closer is more problematic
    X = np.linspace(0+eps,np.pi-eps,N);
    # sweep de theta = [0, pi]
    for n in range(N):
        x = X[n]
        rmse_n = []
        for i in range(M):
            xi = np.random.randn(3)
            xi = xi / np.linalg.norm(xi) * x
            e = mrob.SO3(xi).ln() - xi
            e2 = e.dot(e)
            # handle negatives, since the convention in MROB is positive angles to disambiguate
            if e2 > np.pi :
                e = mrob.SO3(xi).ln() + xi
                e2 = e.dot(e)
                #print('error = ', e, ', ln = ', mrob.SO3(xi).ln() , 'xi = ' , xi)
            rmse_n.append( e2 )
            error[n,i] = np.sqrt(e2)
        rmse[n] = np.sqrt( sum(rmse_n)/M )
        print('Evaluation at theta (angle) = ', x, ' error = ', rmse[n] )
    # calculate RMSE and some statistical perceptinles
    plt.figure(figsize=(8,5))
    plt.plot(X,np.log10(rmse))
    plt.title('RMSE in SO(3)')
    plt.xlabel('angle theta of rotations')
    #plt.savefig('error_SO3.pdf', bbox_inches='tight')
    plt.show()

    # saving data for comparison
    output = open('SO3_domain.pkl', 'wb')
    pickle.dump((rmse, X), output)
    output.close()

    # How to read
    #pkl_file = open('data.pkl', 'rb')
    #rmse, X = pickle.load(pkl_file)
    #pkl_file.close()

# Exp/ln chain SO(3)
# ======================================================
# 1) round pi,0 trajectory
if 1:
    N, M = 120, 100
    
    eps = np.zeros(N)
    eps[0] = 1e0
    for n in range (1,N):
        eps[n] = eps[n-1]*0.8
    rmse = np.zeros(N)
    
    for n in range(N):
        error = []
        for i in range(M):
            xi = np.random.randn(3) 
            xi[0:3] = xi[0:3] / np.linalg.norm(xi[0:3]) * (eps[n]) #around 0
            e = mrob.SO3(xi).ln() - xi
            error.append( e.dot(e) )
        rmse[n] = np.sqrt( sum(error)/M )
        print('Iteration n = ', n , ', eps = ', eps[n], ', current error = ', rmse[n])
    plt.plot(np.log10(rmse))
    plt.title('RMSE of SO(3) transformation $\epsilon$ to 0')
    #plt.savefig('error_SO3.pdf', bbox_inches='tight')
    # saving data for comparison
    output = open('SO3_zero.pkl', 'wb')
    pickle.dump((rmse, X), output)
    output.close()
    plt.show()

# Exp/ln chain SO(3)
# ======================================================
# 1) round pi trajectory
if 1:
    N, M = 120, 100
    
    eps = np.zeros(N)
    eps[0] = 1e0
    for n in range (1,N):
        eps[n] = eps[n-1]*0.8
    rmse = np.zeros(N)
    
    for n in range(N):
        error = []
        for i in range(M):
            xi = np.random.randn(3) 
            xi[0:3] = xi[0:3] / np.linalg.norm(xi[0:3]) * (eps[n]) #around 0
            #xi[0:3] = xi[0:3] / np.linalg.norm(xi[0:3]) * (np.pi - eps[n]) #around pi
            e = mrob.SO3(xi).ln() - xi
            #if e.dot(e) > np.pi :
            #    e = mrob.SO3(xi).ln() + xi
            error.append( e.dot(e) )
        #eps[] *= 0.5
        rmse[n] = np.sqrt( sum(error)/M )
        print('Iteration n = ', n , ', eps = ', eps[n], ', current error = ', rmse[n])
    plt.plot(np.log10(rmse))
    plt.title('RMSE of SO(3) transformation $\epsilon$ to $\pi$')
    # saving data for comparison
    output = open('SO3_pi.pkl', 'wb')
    pickle.dump((rmse, X), output)
    output.close()
    plt.show()



# Exp-Ln SE(3)
# ======================================================
# Overall error for differente segments of theta (angle)
if 1:
    print('Overall evaluation of SE(3) ranging from theta [0,pi] and translations in [-10000,10000]')
    N, M = 200, 100
    error = np.zeros((N,M))
    rmse = np.zeros(N)
    eps = 1e-4 #1e-3 #to avoid the explicit case 0,pi, but closer is more problematic
    X = np.linspace(0+eps,np.pi-eps,N);
    # sweep de theta = [0, pi]
    for n in range(N):
        x = X[n]
        rmse_n = []
        for i in range(M):
            xi = np.random.rand(6) * 10000 # 30000 similar value to kais dataset tranlations.
            xi[0:3] = xi[0:3] / np.linalg.norm(xi[0:3]) * x
            e = mrob.SE3(xi).ln() - xi
            e2 = e.dot(e)
            if e2 > np.pi:
                print('flip sign')
                e = mrob.SE3(xi).mul(mrob.SE3(-xi)).ln()
                e2 = e.dot(e)
            rmse_n.append( e2 )
            error[n,i] = np.sqrt(e2)
        rmse[n] = np.sqrt( sum(rmse_n)/M )
        print('Evaluation at theta (angle) = ', x, ' error = ', rmse[n] )
    # calculate RMSE and some statistical perceptinles
    plt.figure(figsize=(8,5))
    plt.plot(X,np.log10(rmse))
    plt.title('RMSE in SE(3)')
    plt.xlabel('angle theta of rotations')
    #plt.savefig('error_SO3.pdf', bbox_inches='tight')
    plt.show()

    # saving data for comparison
    output = open('SE3_domain.pkl', 'wb')
    pickle.dump((rmse, X), output)
    output.close()

# Qualitative test for some issues found while creating SE3 in mrob
# 2) round 0 trajectory
if 1:
    N, M = 120, 100
    
    eps = np.zeros(N)
    eps[0] = 1e0
    for n in range (1,N):
        eps[n] = eps[n-1]*0.8
    rmse = np.zeros(N)
    
    for n in range(N):
        error = []
        for i in range(M):
            xi = np.random.rand(6) * 10 # 30000 similar value to kais dataset tranlations.
            xi[0:3] = xi[0:3] / np.linalg.norm(xi[0:3]) * (eps[n]) #around 0
            e = mrob.SE3(xi).ln() - xi
            e2 = e.dot(e)
            error.append( e2 )
        #eps[] *= 0.5
        rmse[n] = np.sqrt( sum(error)/M )
        print('Iteration n = ', n , ', eps = ', eps[n], ', current error = ', rmse[n])
    plt.plot(np.log10(rmse))
    plt.title('RMSE of SE(3) transfomration $\epsilon$ close to 0')
    #plt.savefig('error_hard.pdf', bbox_inches='tight')
    plt.show()
    # saving data for comparison
    output = open('SE3_zero.pkl', 'wb')
    pickle.dump((rmse, X), output)
    output.close()


# Qualitative test for some issues found while creating SE3 in mrob
# 3) round pi trajectory
if 1:
    N, M = 120, 100
    
    eps = np.zeros(N)
    eps[0] = 1e0
    for n in range (1,N):
        eps[n] = eps[n-1]*0.8
    rmse = np.zeros(N)
    
    for n in range(N):
        error = []
        for i in range(M):
            xi = np.random.rand(6) * 10 # 30000 similar value to kais dataset tranlations.
            xi[0:3] = xi[0:3] / np.linalg.norm(xi[0:3]) * (np.pi - eps[n]) #around pi
            e = mrob.SE3(xi).ln() - xi
            # To be sure that close to pi we are in our convention, we will provide positive angles by
            e2 = e.dot(e)
            # this error is due to a flip on sign, which does not affect the transformation, only the manifolf parameters
            # we correct this by 
            if e2 > np.pi:
                print('flip sign')
                e = mrob.SE3(xi).mul(mrob.SE3(-xi)).ln()
                e2 = e.dot(e)
            error.append( e2 )
        rmse[n] = np.sqrt( sum(error)/M )
        print('Iteration n = ', n , ', eps = ', eps[n], ', current error = ', rmse[n])
    plt.plot(np.log10(rmse))
    plt.title('RMSE of SE(3) transfomration $\epsilon$ close to $\pi$')
    #plt.savefig('error_hard.pdf', bbox_inches='tight')
    plt.show()
    # saving data for comparison
    output = open('SE3_pi.pkl', 'wb')
    pickle.dump((rmse, X), output)
    output.close()

# 2) Life of objects, some data gets corrupted when chaining functions/operators.
# XXX: WORKING WELL
if 0:
    xi = np.random.rand(6)
    T1 = mrob.SE3(xi)
    T2 = mrob.SE3(np.random.rand(6))
    print(T1.T(), '\n and directly does not work, T = \n', mrob.SE3(xi).inv().T() , '\n is SE3? ', mrob.isSE3(mrob.SE3(xi).inv().T()))
    
    T3 = mrob.SE3(T1.T() @ T2.inv().T())
    T3.print()
    T4 = T1.mul(T2.inv()).mul(T3)
    T4.print()
    print ('is numpy muls a SE3? ', mrob.isSE3(T3.T()), '\n is mul a SE3?', mrob.isSE3(T4.T()))
