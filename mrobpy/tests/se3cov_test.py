import numpy as np
import mrob
import time

import pytest

from mrobpy.examples.test_utils import compound_2nd, compound_4th

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


class TestSE3CovCompoundGeneralCase:
    # initial pose and covariance
    xi_1 = np.array([1,2,-1,0.5,-1,3])
    pose_1 = mrob.geometry.SE3(xi_1)
    covariance_1 = np.diag([0.01,0.02,0.03,0.01,0.05,1])

    xi_2 = np.array([-1,0.2,-1.5,-1.0,-10,-4])
    pose_2 = mrob.geometry.SE3(xi_2)
    covariance_2 = np.diag([0.1,0.1,0.2,0.01,0.01,0.1])

    gt_pose = np.array([[-0.7079330997016517  , -0.6837823830436842  ,  0.17683998129922926 , -0.6167626040635867  ],
       [-0.7051530118471613  ,  0.6984259413063296  , -0.12231285457074748 , -7.846417457327979   ],
       [-0.039874255224246924, -0.21128856369777382 , -0.9766100483923171  ,  8.341550757437583   ],
       [ 0.                  ,  0.                  ,  0.                  ,  1.                  ]])

    sigma_gt_2nd = np.array([[ 0.1151107300776825  , -0.019227083424228932, -0.010736512880775529,  0.04013862360144943 ,  0.08971494057013261 ,  0.23229547731328354 ],
                    [-0.019227083424228932,  0.19233423236663996 ,  0.04039184729113884 , -0.22562722554573875 , -0.056782432343283964,  0.1348639333621249  ],
                    [-0.010736512880775526,  0.04039184729113884 ,  0.15255503755567773 , -0.3245405281697261  , -0.13676562318715507 ,  0.0166438087418345  ],
                    [ 0.04013862360144944 , -0.22562722554573872 , -0.32454052816972606 ,  0.9725717416077854  ,  0.3536034881338836  , -0.15028202457329393 ],
                    [ 0.0897149405701326  , -0.05678243234328397 , -0.13676562318715507 ,  0.35360348813388365 ,  0.3357308049531596  ,  0.1922097626238213  ],
                    [ 0.23229547731328354 ,  0.1348639333621249  ,  0.01664380874183451 , -0.15028202457329395 ,  0.1922097626238213  ,  1.730000798568441   ]])

    sigma_gt_4th = np.array([[ 0.11564860714102614 , -0.01898674488142607 , -0.0106470419401024  ,  0.039528383578574314,  0.09062102246996999 ,  0.23229267256299657 ],
                    [-0.01898674488142607 ,  0.1915211696615563  ,  0.04022354792742576 , -0.22483440388338105 , -0.056250508369940895,  0.13441597165910535 ],
                    [-0.010647041940102396,  0.04022354792742576 ,  0.15151142678698273 , -0.3237690146623021  , -0.13630973777653121 ,  0.016702545778983537],
                    [ 0.039528383578574314, -0.22483440388338102 , -0.32376901466230207 ,  1.0144673094022802  ,  0.3530100712508114  , -0.14929818437639442 ],
                    [ 0.09062102246996998 , -0.0562505083699409  , -0.13630973777653121 ,  0.35301007125081146 ,  0.36755707836164986 ,  0.19404132162170293 ],
                    [ 0.23229267256299657 ,  0.13441597165910535 ,  0.016702545778983547, -0.14929818437639444 ,  0.19404132162170293 ,  1.6873818729288543  ]])

    def test_compound_2nd_order(self):
        cov = mrob.geometry.SE3Cov(self.pose_1, self.covariance_1)
        print('Initial pose and covariance')
        print(cov.T())
        print(cov.cov())

        new_cov = cov.compound_2nd_order(self.pose_2, self.covariance_2)

        print('Updated pose and covariance')
        print(new_cov.T())
        print(new_cov.cov())

        print('Expected pose and covariance')
        print(self.gt_pose)
        print(self.sigma_gt_2nd)

        assert(np.ndarray.all(np.isclose(self.gt_pose, new_cov.T(),atol=1e-10)))
        assert(np.ndarray.all(np.isclose(new_cov.cov(), self.sigma_gt_2nd,atol=1e-10)))

    def test_compound_4th_order(self):
        cov = mrob.geometry.SE3Cov(self.pose_1, self.covariance_1)
        print('Initial pose and covariance')
        print(cov.T())
        print(cov.cov())

        new_cov = cov.compound_4th_order(self.pose_2, self.covariance_2)

        print('Updated pose and covariance')
        print(new_cov.T())
        print(new_cov.cov())

        print('Expected pose and covariance')
        print(self.gt_pose)
        print(self.sigma_gt_2nd)

        assert(np.ndarray.all(np.isclose(self.gt_pose, new_cov.T(),atol=1e-10)))
        assert(np.ndarray.all(np.isclose(new_cov.cov(), self.sigma_gt_4th,atol=1e-10)))

class TestSE3CovCompoundSimpleCase:
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

        assert(np.ndarray.all(np.isclose(c3.T(),self.gt_pose,atol=1e-10)))
        assert(np.ndarray.all(np.isclose(c3.cov(),self.gt_covariance_2nd,atol=1e-10)))

    def test_compound_2nd_order(self):
        cov = mrob.geometry.SE3Cov(self.pose_1, self.covariance_1)
        print('Initial pose and covariance')
        print(cov.T())
        print(cov.cov())

        new_cov = cov.compound_2nd_order(self.pose_2, self.covariance_2)

        print('Updated pose and covariance')
        print(new_cov.T())
        print(new_cov.cov())

        assert(np.ndarray.all(np.isclose(self.gt_covariance_2nd, new_cov.cov(),atol=1e-10)))
        assert(np.ndarray.all(np.isclose(self.gt_pose,new_cov.T(),atol=1e-10)))

    def test_compound_4th_order(self):
        cov = mrob.geometry.SE3Cov(self.pose_1, self.covariance_1)
        print('Initial pose and covariance')
        print(cov.T())
        print(cov.cov())

        new_cov = cov.compound_4th_order(self.pose_2, self.covariance_2)

        print('Updated pose and covariance')
        print(cov.T())
        print(cov.cov())

        print('Expected covariance')
        print(self.gt_covariance_4th)
        print('Expected pose')
        print(self.gt_pose)

        print("Diff norm", np.linalg.norm((new_cov.cov() - self.gt_covariance_4th)))

        assert(np.ndarray.all(np.isclose(self.gt_pose,new_cov.T(),atol=1e-10)))
        assert(np.ndarray.all(np.isclose(self.gt_covariance_4th,new_cov.cov(),atol=1e-10)))

class TestSE3covTimeBenchmarks:
    def test_time_benchmark_2nd_order(self):
        print('Second order compound time benchmark')
        cycles = 10000

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

    def test_time_benchmark_4th_order(self):
        print('Fourth order compound time benchmark')
        cycles = 10000

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

class TestSE3CovMulOperator:
    # initial pose and covariance
    xi_1 = np.array([1,2,-1,0.5,-1,3])
    pose_1 = mrob.geometry.SE3(xi_1)
    covariance_1 = np.diag([0.01,0.02,0.03,0.01,0.05,1])

    xi_2 = np.array([-1,0.2,-1.5,-1.0,-10,-4])
    pose_2 = mrob.geometry.SE3(xi_2)
    covariance_2 = np.diag([0.1,0.1,0.2,0.01,0.01,0.1])

    def test_mul_operator(self):
        cov_1 = mrob.geometry.SE3Cov(self.pose_1, self.covariance_1)
        cov_2 = mrob.geometry.SE3Cov(self.pose_2, self.covariance_2)

        assert(np.ndarray.all(np.isclose((cov_1.mul(cov_2)).cov(),(cov_1.compound_2nd_order(cov_2)).cov(),atol=1e-10)))
