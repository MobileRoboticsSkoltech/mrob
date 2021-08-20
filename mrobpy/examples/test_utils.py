import numpy as np

import mrob

def get_mc(T, sigma, mean=[0,0,0,0,0,0], N = 100000):
    
    xi = np.random.multivariate_normal(mean,sigma,N,check_valid='ignore')

    propagated = []
    for i in range(len(xi)):
        tmp = mrob.geometry.SE3(T)
        tmp.update_lhs(xi[i])
        propagated.append(tmp)

    poses = np.array([x.t() for x in propagated])
    poses = poses.reshape((-1,3))

    xi = np.array([x.Ln() for x in propagated])
    return poses, xi

def compound_mc(T_1, sigma_1, T_2, sigma_2, M = 10000):

    # generating distributions
    p1, xi1 = get_mc(T_1, sigma_1, N=M)

    p2, xi2 = get_mc(T_2, sigma_2, N=M)

    # mean pose
    T = T_1.mul(T_2)

    sigma_mc = np.zeros_like(sigma_1)

    xi_m = []

    for i in range(xi1.shape[0]):

        T_r = mrob.geometry.SE3(xi2[i])
        T_l = mrob.geometry.SE3(xi1[i])

        T_m = T_l.mul(T_r)

        xi_m.append(T_m.mul(T.inv()).Ln())

    # calculating cavirance of xi points coordinates
    sigma_mc = np.cov(np.array(xi_m).transpose())
    return T, sigma_mc


# method to do block matrix permutations
def sigma_permute(sigma):
    s = np.zeros_like(sigma)
    # top left block
    s[:3,:3] = sigma[3:,3:]

    # top right block
    s[:3,3:] = sigma[3:,:3]

    # bottom right block
    s[3:,3:] = sigma[:3,:3]

    # bottom left block
    s[3:,:3] = sigma[:3,3:]

    return s

def compound_2nd(T_1, sigma_1, T_2, sigma_2):
    T = T_1.mul(T_2)
    T_1_adj = sigma_permute(T_1.adj())

    sigma_2_ = T_1_adj@sigma_permute(sigma_2)@T_1_adj.transpose()

    sigma = sigma_permute(sigma_1) + sigma_2_
    sigma = sigma_permute(sigma)

    return T, sigma

def op1(A):
    return -np.eye(A.shape[0])*A.trace() + A

def op2(A,B):
    return op1(A)@op1(B) + op1(B@A)

def compound_4th(T_1, sigma_1, T_2, sigma_2):
    # applying 2nd order to get part of 4th order decomposition
    T, sigma = compound_2nd(T_1, sigma_1, T_2, sigma_2)

    T_1_adj = sigma_permute(T_1.adj())

    _sigma_2 = T_1_adj @ sigma_permute(sigma_2) @ T_1_adj.transpose()

    # map all sigma into [rho, phi] order
    tmp = sigma_permute(sigma)
    s1 = sigma_permute(sigma_1)
    s2 = _sigma_2

    # using equations from reference paper
    sigma_1_rho_rho = s1[:3, :3]
    sigma_1_rho_phi = s1[:3, 3:]
    sigma_1_phi_phi = s1[3:, 3:]

    _sigma_2_rho_rho = s2[:3, :3]
    _sigma_2_rho_phi = s2[:3, 3:]
    _sigma_2_phi_phi = s2[3:, 3:]

    A_1 = np.zeros((6,6))
    A_1[:3, :3] = op1(sigma_1_phi_phi)
    A_1[:3, 3:] = op1(sigma_1_rho_phi + sigma_1_rho_phi.transpose())
    A_1[3:, 3:] = op1(sigma_1_phi_phi)

    _A_2 = np.zeros((6,6))
    _A_2[:3, :3] = op1(_sigma_2_phi_phi)
    _A_2[:3, 3:] = op1(_sigma_2_rho_phi + _sigma_2_rho_phi.transpose())
    _A_2[3:, 3:] = op1(_sigma_2_phi_phi)

    B_rho_rho = op2(sigma_1_phi_phi, _sigma_2_rho_rho) + \
                op2(sigma_1_rho_phi.transpose(), _sigma_2_rho_phi) + \
                op2(sigma_1_rho_phi,_sigma_2_rho_phi.transpose()) + \
                op2(sigma_1_rho_rho, _sigma_2_phi_phi)

    B_rho_phi = op2(sigma_1_phi_phi,_sigma_2_rho_phi.transpose()) + \
                op2(sigma_1_rho_phi,_sigma_2_phi_phi) # there was an error in Barfoot's paper and book

    B_phi_phi = op2(sigma_1_phi_phi, _sigma_2_phi_phi)

    B = np.zeros_like(sigma_1)
    B[:3,:3] = B_rho_rho
    B[:3,3:] = B_rho_phi
    B[3:,:3] = B_rho_phi.transpose()
    B[3:,3:] = B_phi_phi

    tmp += 1./12*(A_1@s2 + \
                   s2@A_1.transpose() + \
                   _A_2@s1 + \
                   s1@ _A_2.transpose()) + 0.25*B

    # mapping resulting sigma into [phi,rho] order
    res = sigma_permute(tmp)

    return T, res
