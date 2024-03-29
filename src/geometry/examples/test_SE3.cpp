#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <iostream>
#include <Eigen/LU>
#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp"
#include "mrob/SO3.hpp"

using namespace std;

TEST_CASE("SO3 tests")
{
    SECTION("Testing the Identity element")
    {
        mrob::SO3 R;
        R.print();
        std::cout << "Identity element error= " << R.ln_vee().norm() << std::endl;
        REQUIRE(R.ln_vee().norm() == Approx(0.0));
    }

    SECTION("Testing the Identity element plus epsilon")
    {
        mrob::Mat31 w;
        w << 1e-15, 0, 0;
        mrob::SO3 R = mrob::SO3(w);
        std::cout << "Identity element error epsilon = " << R.ln_vee().norm() << std::endl;
        REQUIRE(R.ln_vee().norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Testing Regular")
    {
        mrob::Mat31 w;
        w << 1.2, -0.3, 0.2;
        mrob::SO3 R = mrob::SO3(w);
        std::cout << "Exponent and Log test " << (R.ln_vee() - w).norm() << std::endl;
        REQUIRE((R.ln_vee() - w).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("testing bool function isSO3")
    {
        mrob::Mat31 w;
        w << 1.2, -0.3, 0.2;
        mrob::SO3 R(w);
        std::cout << "Matris is SO3: " << mrob::isSO3(R.R()) << std::endl;
        REQUIRE(mrob::isSO3(R.R()) == true);
        mrob::Mat3 notR = mrob::Mat3::Random();
        std::cout << "Matris is SO3: " << mrob::isSO3(notR) << std::endl;
        REQUIRE(mrob::isSO3(notR) == false);
    }

    SECTION("Testing Pi. 1 component")
    {
        mrob::Mat31 w;
        w << M_PI, 0.0, 0.0;
        mrob::SO3 R = mrob::SO3(w);
        //R.print();
        std::cout << "Pi rotation 1 component = " << (R.ln_vee() - w).norm() << std::endl;
        REQUIRE((R.ln_vee() - w).norm() == Approx(0.0));
    }

    SECTION("Testing Pi. 3 components")
    {
        mrob::Mat31 w;
        w << M_PI * std::sqrt(1.0 / 3), M_PI * std::sqrt(1.0 / 3), M_PI * std::sqrt(1.0 / 3);
        mrob::SO3 R = mrob::SO3(w);
        std::cout << "Pi rotation 3 components= " << (R.ln_vee() - w).norm() << std::endl;
        REQUIRE((R.ln_vee() - w).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Testing operators")
    {
        std::cout << "\ntesting operators\n";
        mrob::Mat31 w;
        w << M_PI, 0.0, 0.0;
        mrob::SO3 R = mrob::SO3(w);
        R.print();
        mrob::Mat3 w_hat = R.ln();
        std::cout << w_hat << std::endl;
        REQUIRE((w - mrob::vee3(w_hat)).norm() == Approx(0.0));
    }

    SECTION("Testing inverse")
    {
        mrob::Mat31 w;
        w << M_PI, 0.0, 0.0;
        mrob::SO3 R = mrob::SO3(w);

        std::cout << "testing inverse" << std::endl;
        mrob::SO3 Rt = R.inv();
        std::cout << "invers = " << Rt.R() << std::endl;
        REQUIRE((Rt.mul(R).R() - mrob::Mat3::Identity()).norm() == Approx(0.0));
    }
}

TEST_CASE("SE3 tests")
{
    SECTION("Testing the constructor. Default")
    std::cout << "\n\nSE3 tests" << std::endl;
    {
        mrob::SE3 T;
        //T.print();
        std::cout << "Identity element error= " << T.ln_vee().norm() << std::endl;
        REQUIRE(T.ln_vee().norm() == Approx(0.0));
    }

    SECTION("Testing the constructor. Xi argument. 1 case")
    {
        mrob::Mat61 xi;
        xi << 1e-9, 0, 0, 20, 100, 4;
        mrob::SE3 T(xi);
        T.print();
        std::cout << "random element plus epsilon= " << (T.ln_vee() - xi).norm() << std::endl;
        REQUIRE((T.ln_vee() - xi).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Testing the constructor. Xi argument. 2 case")
    {
        mrob::Mat61 xi;
        xi << 1, 0, -0.2, 5, 10, 2;
        mrob::SE3 T(xi);
        std::cout << "Some normal element error = " << (T.ln_vee() - xi).norm() << std::endl;
        REQUIRE((T.ln_vee() - xi).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Testing bool function isSE3")
    {
        mrob::Mat61 xi;
        xi << 1, 0, -0.2, 5, 10, 2;
        mrob::SE3 T(xi);
        std::cout << "input SE3 and Matris is SE3?: " << mrob::isSE3(T.T()) << std::endl;
        REQUIRE(mrob::isSE3(T.T()) == true);
        mrob::Mat4 notT = mrob::Mat4::Random();
        std::cout << "Random Matris is SE3: " << mrob::isSE3(notT) << std::endl;
        REQUIRE(mrob::isSE3(notT) == false);
    }

    SECTION("Testing Pi")
    {
        mrob::Mat61 xi;
        xi << M_PI, 0, 0, 5, 100, 2;
        mrob::SE3 T(xi);
        T.print_lie();
        std::cout << "Pi error plus trans= " << (T.ln_vee() - xi).norm() << std::endl;
        REQUIRE((T.ln_vee() - xi).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Testin Pi. 3 components")
    {
        mrob::Mat61 xi;
        xi << M_PI * std::sqrt(1.0 / 3) - 1e-4, M_PI * std::sqrt(1.0 / 3), M_PI * std::sqrt(1.0 / 3), 5, 1000, 200;
        mrob::SE3 T(xi);
        T.print_lie();
        std::cout << "Pi error 3 comp plus trans= " << (T.ln_vee() - xi).norm() << std::endl;
        REQUIRE((T.ln_vee() - xi).norm() == Approx(0.0).margin(1e-10));
    }

    SECTION("Testing update")
    {
        mrob::SE3 T1;
        T1.print();
        T1.print_lie();
        mrob::Mat61 xi;
        xi << 1, 0, -0.2, 5, 10, 2;
        mrob::SE3 T(xi);
        T.print();
        xi << T.ln_vee();
        mrob::SE3 T2(xi);
        T2.print();
        std::cout << "Matrix distance = " << (T.T() - T2.T()).norm() << std::endl;
        REQUIRE((T.T() - T2.T()).norm() == Approx(0.0).margin(1e-12));

        std::cout << "testing update\n";
        T2.update_lhs(xi);
        T2.print();
        mrob::Mat41 v;
        v << 1.0, 3.2, -1.2, 1.0;
        std::cout << T2.T() * v << std::endl;
        mrob::Mat41 tmp = T2.T() * v;
        v = T2.T() * v;
        std::cout << v << std::endl;
        REQUIRE((v - tmp).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Access inner matrix")
    {
        mrob::SE3 T;
        mrob::Mat4 gt;
        gt << 1, 0, 0, 1,
            0, 1, 0, 2,
            0, 0, 1, 3,
            0, 0, 0, 1;

        T.ref2T() << gt;

        REQUIRE((T.T() - gt).norm() == Approx(0.0).margin(1e-12));
        T.print();
    }

    SECTION("Testing inverse()")
    {
        mrob::Mat61 xi;
        xi << 1, 0, -0.2, 5, 10, 2;
        mrob::SE3 T(xi);
        T.print();
        xi << T.ln_vee();
        mrob::SE3 T2(xi);
        std::cout << "testing inverse" << std::endl;
        mrob::SE3 Tt = T2.inv();
        std::cout << "invers = " << Tt.T() << std::endl;
        std::cout << "Matrix distance = " << (Tt.T() - T2.T().inverse()).norm() << std::endl;
        REQUIRE((Tt.T() - T2.T().inverse()).norm() == Approx(0.0).margin(1e-12));

        mrob::Mat61 xi2 = -T2.ln_vee();
        mrob::SE3 T22(xi2);
        std::cout << "Matrix distance by negating= " << (T22.T() - T2.T().inverse()).norm() << std::endl;
        REQUIRE((T22.T() - T2.T().inverse()).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Testing adj()")
    {
        mrob::Mat61 xi;
        xi << 0, 0, 0, 1, 2, 3;
        mrob::SE3 T(xi);

        mrob::Mat6 gt;
        gt << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, -3, 2, 1, 0, 0,
            3, 0, -1, 0, 1, 0,
            -2, 1, 0, 0, 0, 1;

        std::cout << "testing adjoint" << std::endl;
        std::cout << "Adjoint= " << T.adj() << std::endl;
        REQUIRE((T.adj() - gt).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Testing adjoint property: T*exp(xi) = exp(Adj_t(xi))*Ts")
    {
        // checking equality for Adj(): T*exp(xi) = exp(Adj_t(xi))*T
        //initial pose
        mrob::Mat61 xi;
        xi << 0.1, 0.2, 0.3, 4, 5, 6;
        mrob::SE3 T(xi);

        //small disturbance
        mrob::Mat61 eta;
        eta << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1;

        // left part of equality
        mrob::SE3 T_exp(T);
        T_exp.update_rhs(eta);

        // right part of the equality
        mrob::SE3 tmp_pose(mrob::Mat4(T.T()));
        tmp_pose.update_lhs(T.adj()*eta);

        // checking that both parts of equality are equal
        REQUIRE((T_exp.T() - tmp_pose.T()).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Testing subblock metrixes")
    {
        mrob::Mat61 xi;
        xi << 0, 0, 0, 1, 2, 3;
        mrob::SE3 T(xi);

        mrob::Mat4 gt;
        gt << 1, 0, 0, 1,
            0, 1, 0, 2,
            0, 0, 1, 3,
            0, 0, 0, 1;

        std::cout << "Subblock methods an matrix:" << std::endl;
        T.print();
        std::cout << "\nT = " << T.T() << std::endl;
        REQUIRE((T.T() - gt).norm() == Approx(0.0).margin(1e-12));

        std::cout << "\nR = " << T.R() << std::endl;
        REQUIRE((T.R() - mrob::Mat3::Identity()).norm() == Approx(0.0).margin(1e-12));

        std::cout << "\nt = " << T.t() << std::endl;
        REQUIRE((T.t() - gt.topRightCorner<3, 1>()).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Testing transformation")
    {
        mrob::Mat61 xi;
        xi << 1, 0, -0.2, 5, 10, 2;
        mrob::SE3 T(xi);
        mrob::Mat31 p, p2;
        p << 2, 3, -1;
        p2 = p;
        p = T.transform(p);
        p = T.inv().transform(p);
        std::cout << "Testing transformations = " << (p - p2).norm() << std::endl;
        REQUIRE((p - p2).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Multiplication")
    {
        mrob::SE3 T1;
        T1.ref2T() <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1;

        mrob::SE3 T2;
        T2.ref2T() <<
        1,0,0,1,
        0,1,0,2,
        0,0,1,3,
        0,0,0,1;

        mrob::Mat4 gt;
        gt <<
        1,0,0,1,
        0,1,0,2,
        0,0,1,3,
        0,0,0,1;

        REQUIRE((T1.mul(T2).T() - gt).norm() == Approx(0.0).margin(1e-12));
    }
}
