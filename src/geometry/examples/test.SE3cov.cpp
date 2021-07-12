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

    SECTION("Test compound 2nd order w/ and w/o notation transform")
    {
        Mat61 xi;
        xi << 0.1, 0.2, 0.3, 1, 2, 3;
        mrob::SE3 pose_increment(xi);

        xi << 0,0,0,0,0,0;
        mrob::SE3 initial_pose(xi);

        Mat6 increment_covariance;
        increment_covariance.diagonal() << 0.1, 0.2, 0.3, 0.2, 0.2, 0.2;

        Mat6 initial_covariance;
        initial_covariance << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

        mrob::SE3Cov uncertainty(initial_pose, initial_covariance);
        Mat6 adj = uncertainty.adj();
        // Mat4 T = cov.T() * pose_increment.T();

        Mat6 with_transform = mrob::SE3Cov::notation_transform(mrob::SE3Cov::notation_transform(uncertainty.cov())
                                + adj*mrob::SE3Cov::notation_transform(increment_covariance)*adj.transpose());

        Mat6 without_transform = uncertainty.cov() + adj*increment_covariance*adj.transpose();

        REQUIRE((with_transform - without_transform).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Notation transform")
    {
        Mat6 cov(Mat6::Zero());
        cov.diagonal() << 1,2,3,4,5,6;

        REQUIRE((mrob::SE3Cov::notation_transform(mrob::SE3Cov::notation_transform(cov)) == cov));   
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

    SECTION("compound 2nd order")
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
        gt_pose <<  0.0707372 , -0.99749499,  0.        ,  1.16499666,
                    0.99749499,  0.0707372 ,  0.        ,  0.61950853,
                    0.        ,  0.        ,  1.        ,  0.        ,
                    0.        ,  0.        ,  0.        ,  1.        ;
        
        REQUIRE((uncertainty.T() - gt_pose).norm() ==Approx(0.0).margin(1e-8));
    }

    SECTION("compaund 4th order")
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
