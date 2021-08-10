import numpy as np
import mrob
import time

import pytest

import mrobpy.examples.utils
from mrobpy.examples.utils import compound_2nd, compound_4th, compound_mc

class TestSE3CovConstructors:
    def test_se3cov_default_constructor(self):
        cov = mrob.geometry.SE3Cov()
        
        assert(np.ndarray.all(cov.T() == np.eye(4)))
        assert(np.ndarray.all(cov.cov() == np.eye(6)))

    def test_se3cov_constructor(self):
        xi = np.array([0.1, 0.2, 0.3, 1, 2, 3])
        pose = mrob.geometry.SE3(xi)

        covariance = np.diag([0.5, 0.6, 0.7, 0.8, 0.9, 1.0])

        cov = mrob.geometry.SE3Cov(pose, covariance)

        assert(np.ndarray.all(cov.T() == pose.T()))
        assert(np.ndarray.all(cov.cov() == covariance))


class TestSE3CovCompoundComplex:
    # initial pose and covariance
    xi_1 = np.array([0,0,0,0.5,0,0])
    pose_1 = mrob.geometry.SE3(xi_1)
    covariance_1 = np.diag([0,0,0.01,0.01,0.01,0])

    xi_2 = np.array([0,0,1.5,1.0,0,0])
    pose_2 = mrob.geometry.SE3(xi_2)
    covariance_2 = np.diag([0,0,0.1,0.01,0.01,0])
    
    # monte carlo as reference
    T_gt,sigma_gt = compound_mc(pose_1, covariance_1, pose_2, covariance_2, M=100_000)

    sigma_gt_2nd = np.array([
        0.    , 0.    , 0.    , 0.    , 0.    , 0.,
        0.    , 0.    , 0.    , 0.    , 0.    , 0.,
        0.    , 0.    , 0.1125, 0.    , 0.005 , 0.,
        0.    , 0.    , 0.    , 0.02  , 0.    , 0.,
        0.    , 0.    , 0.005 , 0.    , 0.02  , 0.,
        0.    , 0.    , 0.    , 0.    , 0.    , 0.
    ]).reshape((6,6))

    def test_compound_1(self):
        cov = mrob.geometry.SE3Cov(self.pose_1, self.covariance_1)
        print('Initial pose and covariance')
        print(cov.T())
        print(cov.cov())

        cov.compound_2nd_order(self.pose_2, self.covariance_2)

        print('Updated pose and covariance')
        print(cov.T())
        print(cov.cov())

        print('Expected pose and covariance')
        print(self.T_gt.T())
        print(self.sigma_gt)

        # assert(np.ndarray.all(self.sigma_gt == cov.cov()))
        # assert(np.ndarray.all(self.T_gt.T() == cov.T()))
        # assert(np.ndarray.all(cov.cov() == self.sigma_gt_2nd))



class TestSE3CovCompoundSimple:
    # initial pose and covariance
    xi_1 = np.array([0,0,0,0.5,0,0])
    pose_1 = mrob.geometry.SE3(xi_1)
    covariance_1 = np.diag([0,0,0.01,0.01,0.01,0])

    #incremental pose and covariance
    xi_2 = np.array([0,0,1.5,1.0,0,0])
    pose_2 = mrob.geometry.SE3(xi_2)
    covariance_2 = np.diag([0,0,0.1,0.01,0.01,0])

    # expected pose matrix
    gt_pose = np.array([
        [ 0.070737201667703 , -0.9974949866040546,  0.,  1.1649966577360362],
        [ 0.9974949866040546,  0.070737201667703 ,  0.,  0.6195085322215313],
        [ 0.                ,  0.                ,  1.,  0.                ],
        [ 0.                ,  0.                ,  0.,  1.                ]], dtype='float64')

    # expected covariance matrix after 2nd order
    gt_covariance_2nd = np.array([
        [ 0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ],
        [ 0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ],
        [ 0.   ,  0.   ,  0.11 ,  0.   , -0.05 ,  0.   ],
        [ 0.   ,  0.   ,  0.   ,  0.02 ,  0.   ,  0.   ],
        [ 0.   ,  0.   , -0.05 ,  0.   ,  0.045,  0.   ],
        [ 0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ]])

    gt_covariance_4th = np.array([
        [ 0.,  0.,  0.  ,  0.                ,  0.     ,  0.],
        [ 0.,  0.,  0.  ,  0.                ,  0.     ,  0.],
        [ 0.,  0.,  0.11,  0.                , -0.05   ,  0.],
        [ 0.,  0.,  0.  ,  0.0201541666666667,  0.     ,  0.],
        [ 0.,  0., -0.05,  0.                ,  0.04505,  0.],
        [ 0.,  0.,  0.  ,  0.                ,  0.     ,  0.]])

    def test_mul_operator(self):
        c1 = mrob.geometry.SE3Cov(self.pose_1, self.covariance_1)
        c2 = mrob.geometry.SE3Cov(self.pose_2, self.covariance_2)

        c3 = c1*c2 # mul operator corresponds to 2nd order compound

        print(c3.T())
        print(c3.cov())

        assert(np.ndarray.all(c3.T() == self.gt_pose))
        assert(np.ndarray.all(c3.cov() == self.gt_covariance_2nd))

    def test_compound_1(self):
        cov = mrob.geometry.SE3Cov(self.pose_1, self.covariance_1)
        print('Initial pose and covariance')
        print(cov.T())
        print(cov.cov())

        cov.compound_2nd_order(self.pose_2, self.covariance_2)

        print('Updated pose and covariance')
        print(cov.T())
        print(cov.cov())

        assert(np.ndarray.all(self.gt_covariance_2nd == cov.cov()))
        assert(np.ndarray.all(self.gt_pose == cov.T()))

    def test_compound_2(self):
        cov = mrob.geometry.SE3Cov(self.pose_1, self.covariance_1)
        print('Initial pose and covariance')
        print(cov.T())
        print(cov.cov())

        cov.compound_4th_order(self.pose_2, self.covariance_2)

        print('Updated pose and covariance')
        print(cov.T())
        print(cov.cov())

        print('Expected covariance')
        print(self.gt_covariance_4th)
        print('Expected pose')
        print(self.gt_pose)

        print("Diff norm", np.linalg.norm((cov.cov() - self.gt_covariance_4th)))

        assert(np.ndarray.all(self.gt_pose == cov.T()))
        assert(np.ndarray.all(np.isclose(self.gt_covariance_4th,cov.cov(),atol=1e-10)))

class TestSE3covTimeBenchmarks:
    def test_time_benchmark_1(self):
        print('Second order compound time benchmark')
        cycles = 1_000_00

        start_time = time.time()

        xi_1 = np.array([0,0,0,0.5,0,0])
        T_1 = mrob.geometry.SE3(xi_1)
        sigma_1 = np.diag([0,0,0.01,0.01,0.01,0])

        xi_2 = np.array([0,0,1.5,1.0,0,0])
        T_2 = mrob.geometry.SE3(xi_2)
        sigma_2 = np.diag([0,0,0.1,0.01,0.01,0])

        for i in range(cycles):
            compound_2nd(T_1, sigma_1, T_2, sigma_2)

        print('pure python:')
        time_pure_python = (time.time() - start_time)
        print("%s cycles --- %s seconds ---" % (cycles, time_pure_python))

        cov = mrob.geometry.SE3Cov(T_1, sigma_1)

        start_time = time.time()

        for i in range(cycles):
            cov.compound_2nd_order(T_2, sigma_2)

        print('c++ bindings:')
        time_cpp_bindings = (time.time() - start_time)
        print("%s cycles --- %s seconds ---" % (cycles, time_cpp_bindings))

        print("Pure python time / c++ bindings time = %s" % (time_pure_python/time_cpp_bindings))

        assert(time_pure_python > time_cpp_bindings )

    def test_time_benchmark_2(self):
        print('Fourth order compound time benchmark')
        cycles = 1_000_0

        start_time = time.time()

        xi_1 = np.array([0,0,0,0.5,0,0])
        T_1 = mrob.geometry.SE3(xi_1)
        sigma_1 = np.diag([0,0,0.01,0.01,0.01,0])

        xi_2 = np.array([0,0,1.5,1.0,0,0])
        T_2 = mrob.geometry.SE3(xi_2)
        sigma_2 = np.diag([0,0,0.1,0.01,0.01,0])

        for i in range(cycles):
            compound_4th(T_1, sigma_1, T_2, sigma_2)

        print('pure python:')
        time_pure_python = (time.time() - start_time)
        print("%s cycles --- %s seconds ---" % (cycles, time_pure_python))

        cov = mrob.geometry.SE3Cov(T_1, sigma_1)

        start_time = time.time()

        for i in range(cycles):
            cov.compound_4th_order(T_2, sigma_2)

        print('c++ bindings:')
        time_cpp_bindings = (time.time() - start_time)
        print("%s cycles --- %s seconds ---" % (cycles, time_cpp_bindings))

        print("Pure python time / c++ bindings time = %s" % (time_pure_python/time_cpp_bindings))

        assert(time_pure_python > time_cpp_bindings )

