#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <iostream>
#include <Eigen/LU>
#include "mrob/matrix_base.hpp"
#include "mrob/SE3cov.hpp"

using namespace std;
using namespace mrob;

TEST_CASE("SE3cov basic tests")
{
    SECTION("SE3Cov constructor with default arguments")
    {
        //TODO fix Constructor with default arguments. for some reason there is a linking error when calling constructor with efault arguments: mrob::SE3Cov cov;
        mrob::SE3Cov cov;
        REQUIRE((cov.T() - Mat4::Identity()).norm() == Approx(0.0).margin(1e-12));
        REQUIRE((cov.cov() - Mat6::Identity()).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("SE3Cov constructor with arguments")
    {
        Mat61 xi;
        xi << 0.1, 0.2, 0.3, 1, 2, 3;
        mrob::SE3 pose(xi);

        Mat6 covariance(Mat6::Identity());
        covariance.diagonal() << 1,2,3,4,5,6;

        mrob::SE3Cov cov(pose,covariance);

        REQUIRE((cov.T() - pose.T()).norm() == Approx(0.0).margin(1e-12));
        REQUIRE((cov.cov() - covariance).norm() == Approx(0.0).margin(1e-12));
    }

    // SECTION("Test compound 2nd order w/ and w/o notation transform")
    // {
    //     Mat61 xi;
    //     xi << 0., 0., 1.5, 1.0, 0, 0;
    //     mrob::SE3 pose_increment(xi);

    //     xi << 0,0,0,0.5,0,0;
    //     mrob::SE3 initial_pose(xi);

    //     Mat6 increment_covariance;
    //     // increment_covariance.diagonal() << 0.0, 0.0, 0.1, 0.01, 0.01, 0.0;
    //     increment_covariance.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;

    //     Mat6 initial_covariance;
    //     // initial_covariance.diagonal() << 0.0, 0.0, 0.01, 0.01, 0.01, 0.0;
    //     initial_covariance.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;

    //     mrob::SE3Cov uncertainty(initial_pose, initial_covariance);
    //     Mat6 adj = uncertainty.adj();

    //     std::cout << adj << std::endl;
    //     std::cout << uncertainty.cov() << std::endl;
    //     std::cout << uncertainty.T() << std::endl;

    //     Mat6 with_transform = mrob::SE3Cov::notation_transform(mrob::SE3Cov::notation_transform(uncertainty.cov())
    //                             + adj*mrob::SE3Cov::notation_transform(increment_covariance)*adj.transpose());

    //     std::cout << with_transform << std::endl;

    //     std::cout << uncertainty.cov() << std::endl;
    //     std::cout << uncertainty.T() << std::endl;
    //     std::cout << adj << std::endl;

    //     Mat6 without_transform = uncertainty.cov() + adj*increment_covariance*adj.transpose();

    //     std::cout << without_transform << std::endl;

    //     REQUIRE((with_transform - without_transform).norm() == Approx(0.0).margin(1e-12));
    // }

    SECTION("Notation transform. Check self-inverse property.")
    {
        Mat6 cov(Mat6::Zero());
        cov.diagonal() << 1,2,3,4,5,6;

        REQUIRE((mrob::SE3Cov::notation_transform(mrob::SE3Cov::notation_transform(cov)) == cov));   
    }

    SECTION("Notation transform. Permutation check.")
    {
        Mat6 cov(Mat6::Zero());
        cov <<   1, 2, 3, 4, 5, 6,
                 7, 8, 9,10,11,12,
                13,14,15,16,17,18,
                19,20,21,22,23,24,
                25,26,27,28,29,30,
                31,32,33,34,35,36;

        Mat6 gt_after(Mat6::Zero());
        gt_after << 22,23,24,19,20,21,
                    28,29,30,25,26,27,
                    34,35,36,31,32,33,
                     4, 5, 6, 1, 2, 3,
                    10,11,12, 7, 8, 9,
                    16,17,18,13,14,15;

        REQUIRE((mrob::SE3Cov::notation_transform(cov) - gt_after).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Default Constructor")
    {
        mrob::SE3Cov uncertainty;
        REQUIRE(uncertainty.T() == Mat4::Identity());
        REQUIRE(uncertainty.cov() == Mat6::Identity());
    }

    SECTION("Constructor")
    {
        mrob::SE3 pose;
        Mat6 cov(Mat6::Identity());
        mrob::SE3Cov uncertainty(pose, cov);

        REQUIRE(uncertainty.T() == pose.T());
        REQUIRE(uncertainty.cov() == cov);
    }

    SECTION("Compounding. 2nd order")
    {
        Mat61 xi;
        xi << 0,0,0,0.5,0,0;
        mrob::SE3 T(xi);
        Mat6 cov(Mat6::Zero());
        cov.diagonal() << 0.0, 0.0, 0.01, 0.01, 0.01, 0.0;

        mrob::SE3Cov uncertainty(T, cov);

        xi << 0, 0, 1.5, 1, 0, 0;
        mrob::SE3 pose_increment(xi);

        Mat6 increment_covariance(Mat6::Zero());
        increment_covariance.diagonal() << 0, 0, 0.1, 0.01, 0.01, 0;
        
        uncertainty.compound_2nd_order(pose_increment, increment_covariance);

        Mat6 gt_cov(Mat6::Zero());
        gt_cov <<   0.    , 0.    , 0.    , 0.    , 0.    , 0.,
                    0.    , 0.    , 0.    , 0.    , 0.    , 0.,
                    0.    , 0.    , 0.1125, 0.    , 0.005 , 0.,
                    0.    , 0.    , 0.    , 0.02  , 0.    , 0.,
                    0.    , 0.    , 0.005 , 0.    , 0.02  , 0.,
                    0.    , 0.    , 0.    , 0.    , 0.    , 0.;

        REQUIRE((uncertainty.cov() - gt_cov).norm() == Approx(0.0).margin(1e-10));

        Mat4 gt_pose(Mat4::Zero());
        gt_pose <<  0.0707372017 , -0.9974949866,  0.        ,  1.1649966577,
                    0.9974949866,  0.0707372017 ,  0.        ,  0.6195085322,
                    0.        ,  0.        ,  1.        ,  0.        ,
                    0.        ,  0.        ,  0.        ,  1.        ;
        
        REQUIRE((uncertainty.T() - gt_pose).norm() == Approx(0.0).margin(1e-8));
    }

    SECTION("Curley wedge operator")
    {
        Mat61 xi;
        xi << 1,2,3,4,5,6;

        Mat6 output = curly_wedge(xi);
        Mat6 gt(Mat6::Zero());

        gt <<    0, -3,  2,  0, -6,  5,
                 3,  0, -1,  6,  0, -4,
                -2,  1,  0, -5,  4,  0,
                 0,  0,  0,  0, -3,  2,
                 0,  0,  0,  3,  0, -1,
                 0,  0,  0, -2,  1,  0;

        std::cout << output << std::endl;
        std::cout << gt << std::endl;

        REQUIRE((output - gt).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Compounding. 4th order")
    {
        Mat61 xi;
        xi << 0,0,0,0.5,0,0;
        mrob::SE3 T(xi);

        Mat6 cov(Mat6::Zero());
        cov.diagonal() << 0.0, 0.0, 0.01, 0.01, 0.01, 0.0;

        mrob::SE3Cov uncertainty(T,cov);

        xi << 0, 0, 1.5, 1, 0, 0;
        mrob::SE3 pose_increment(xi);

        Mat6 increment_covariance(Mat6::Zero());
        increment_covariance.diagonal() << 0, 0, 0.1, 0.01, 0.01, 0;
        
        uncertainty.compound_4th_order(pose_increment, increment_covariance);

        Mat6 gt_cov(Mat6::Zero());

        gt_cov <<   0.        , 0.        , 0.        , 0.        , 0.        , 0.,
                    0.        , 0.        , 0.        , 0.        , 0.        , 0.,
                    0.        , 0.        , 0.1125    , 0.        , 0.005     , 0.,
                    0.        , 0.        , 0.        , 0.02009375, 0.        , 0.,
                    0.        , 0.        , 0.005     , 0.        , 0.02009375, 0.,
                    0.        , 0.        , 0.        , 0.        , 0.        , 0.;

        Mat4 gt_pose(Mat4::Zero());
        gt_pose <<  0.0707372 , -0.99749499,  0.        ,  1.16499666,
                    0.99749499,  0.0707372 ,  0.        ,  0.61950853,
                    0.        ,  0.        ,  1.        ,  0.        ,
                    0.        ,  0.        ,  0.        ,  1.        ;
        
        REQUIRE((uncertainty.T() - gt_pose).norm() ==Approx(0.0).margin(1e-8));
        REQUIRE((uncertainty.cov() - gt_cov).norm() == Approx(0.0).margin(1e-10));
    }
    
}
