import numpy as np
import mrob
import time

import pytest

class TestSO3Basic:
    def test_identity(self):
        R = mrob.geometry.SO3()

        R.print()
        assert(np.linalg.norm(R.Ln()) == 0)

    def test_identity_with_eps(self):
        w = np.array([1e-100, 0, 0])
        R = mrob.geometry.SO3(w)
        print(R.Ln())
        # in python eps term does not work similar to one to c++
        assert(np.linalg.norm(R.Ln() - w) == 0)

    def test_general_case(self):
        w = np.array([1.2, -0.3, 0.2])
        R = mrob.geometry.SO3(w)
        assert(np.isclose(np.linalg.norm(R.Ln() - w),0,atol=1e-15))

    def test_is_so3(self):
        w = np.array([1.2, -0.3, 0.2])
        R = mrob.geometry.SO3(w)
        assert(mrob.geometry.isSO3(R.R()) == True)

        random_mat = np.random.randn(3,3)
        assert(mrob.geometry.isSO3(random_mat) == False)

    def test_pi_1_component(self):
        w = np.array([np.pi,0,0])
        R = mrob.geometry.SO3(w)
        assert(np.isclose(np.linalg.norm(R.Ln() - w),0,atol=1e-15))

    def test_pi_3_components(self):
        w = np.array([np.pi * np.sqrt(1.0 / 3), np.pi * np.sqrt(1.0 / 3), np.pi * np.sqrt(1.0 / 3)])
        R = mrob.geometry.SO3(w)
        assert(np.isclose(np.linalg.norm(R.Ln() - w),0,atol=1e-15))

    # this test is skipped for python
    #     SECTION("Testing operators")
    #     {
    #         std::cout << "\ntesting operators\n";
    #         mrob::Mat31 w;
    #         w << M_PI, 0.0, 0.0;
    #         mrob::SO3 R = mrob::SO3(w);
    #         R.print();
    #         mrob::Mat3 w_hat = R.ln();
    #         std::cout << w_hat << std::endl;
    #         REQUIRE((w - mrob::vee3(w_hat)).norm() == Approx(0.0));
    #     }

    def test_inverse(self):
        w = np.array([np.pi,0,0])
        R = mrob.geometry.SO3(w)

        Rt = R.inv()
        assert(np.isclose(np.linalg.norm(Rt.mul(R).R() - np.eye(3)),0,atol=1e-15))

class TestSE3Basic:
    xi_1 = np.array([1e-9, 0, 0, 20, 100, 4])
    xi_2 = np.array([1, 0, -0.2, 5, 10, 2])
    xi_pi = np.array([np.pi, 0, 0, 5, 100, 2])
    xi_3 = np.pi*np.sqrt(1.0 / 3) - 1e-4, np.pi*np.sqrt(1.0 / 3), np.pi*np.sqrt(1.0 / 3), 5, 1000, 200

    def test_constructor(self):
        T = mrob.geometry.SE3()
        assert(np.linalg.norm(T.Ln()) == 0)

    def test_constructor_xi_arg(self):
        T = mrob.geometry.SE3(self.xi_1)

        assert(np.isclose(np.linalg.norm(T.Ln() - self.xi_1),0,atol=1e-15))

        T = mrob.geometry.SE3(self.xi_2)
        assert(np.isclose(np.linalg.norm(T.Ln() - self.xi_2),0,atol=1e-14))

    def test_is_se3(self):
        T = mrob.geometry.SE3(self.xi_2)
        assert(mrob.geometry.isSE3(T.T()) == True)

        random_mat = np.random.randn(4,4)
        assert(mrob.geometry.isSE3(random_mat) == False)

    def test_pi(self):
        T = mrob.geometry.SE3(self.xi_pi)
        assert(np.isclose(np.linalg.norm(T.Ln() - self.xi_pi),0,atol=1e-14))

    def test_pi_3_components(self):
        T = mrob.geometry.SE3(self.xi_3)
        # accuracy of 1.8e-11 here
        assert(np.isclose(np.linalg.norm(T.Ln() - self.xi_3),0,atol=1e-10))

    def test_update(self):
        T_1 = mrob.geometry.SE3()
        T = mrob.geometry.SE3(self.xi_2)
        xi = T.Ln()
        T_2 = mrob.geometry.SE3(xi)

        assert(np.isclose(np.linalg.norm(T.T() - T_2.T()),0,atol=1e-15))

        T_2.update_lhs(xi)

        v = np.array([1.0, 3.2, -1.2, 1.0])

        tmp = T_2.T()@v

        v = T_2.T()@v

        assert(np.isclose(np.linalg.norm(v - tmp),0,atol=1e-15))

    # SE3.ref2T() does not have a binding in python
    #     SECTION("Access inner matrix")
    #     {
    #         mrob::SE3 T;
    #         mrob::Mat4 gt;
    #         gt << 1, 0, 0, 1,
    #             0, 1, 0, 2,
    #             0, 0, 1, 3,
    #             0, 0, 0, 1;

    #         T.ref2T() << gt;

    #         REQUIRE((T.T() - gt).norm() == Approx(0.0).margin(1e-12));
    #         T.print();
    #     }

    def test_inverse(self):
        T = mrob.geometry.SE3(self.xi_2)

        T_inv = T.inv()

        assert(np.isclose(np.linalg.norm(T_inv.T()@T.T() - np.eye(4)),0,atol=1e-15))

    def test_adjoint_simple_case(self):
        xi = np.array([0,0,0,1,2,3])
        T = mrob.geometry.SE3(xi)

        gt = np.array([1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, -3, 2, 1, 0, 0,
            3, 0, -1, 0, 1, 0,
            -2, 1, 0, 0, 0, 1]).reshape(6,6)

        assert(np.linalg.norm(T.adj() - gt) == 0)

    def test_adjoint_property(self):
        #checking equality for Adj(): T*exp(xi) = exp(Adj_t(xi))*T

        # initial pose
        xi = np.array([0.1, 0.2, 0.3, 4, 5, 6])
        T = mrob.geometry.SE3(xi)

        # small disturbance
        eta = np.array([0.05, 0.05, 0.05, 0.1, 0.1, 0.1])

        T_exp = mrob.geometry.SE3(T)
        T_exp.update_rhs(eta)

        tmp_pose = mrob.geometry.SE3(T.T())
        tmp_pose.update_lhs(T.adj()@eta)

        assert(np.isclose(np.linalg.norm(T_exp.T() - tmp_pose.T()),0,atol=1e-15))

    def test_access_to_submatrices(self):
        xi = np.array([0,0,0,1,2,3])

        T = mrob.geometry.SE3(xi)

        gt = np.array([1, 0, 0, 1,
                        0, 1, 0, 2,
                        0, 0, 1, 3,
                        0, 0, 0, 1]).reshape(4,4)

        assert(np.linalg.norm(T.T() - gt) == 0)
        assert(np.linalg.norm(T.R() - gt[:3,:3]) == 0)
        assert(np.linalg.norm(T.t() - gt[:3,3]) == 0)

    def test_transformation(self):
        T = mrob.geometry.SE3(self.xi_2)

        p = np.array([2,3,-1])
        p_origin = p

        p = T.transform(p)
        p = T.inv().transform(p)
        # accuracy is 1.6e-15 here
        assert(np.isclose(np.linalg.norm(p - p_origin),0,atol=1e-14))

    def test_multiplication_simple(self):
        T_1 = mrob.geometry.SE3()

        T_2 = mrob.geometry.SE3(np.array([0,0,0,1,2,3]))

        gt = np.array([1,0,0,1,
                        0,1,0,2,
                        0,0,1,3,
                        0,0,0,1]).reshape(4,4)
        assert(np.linalg.norm(T_1.mul(T_2).T() - gt) == 0)
