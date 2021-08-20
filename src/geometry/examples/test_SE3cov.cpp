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

    SECTION("Curley wedge operator. MROB notation")
    {
        Mat61 xi;
        xi << 1,2,3,4,5,6;

        Mat6 output = curly_wedge(xi);
        Mat6 gt(Mat6::Zero());

        gt <<    0, -3,  2,  0,  0,  0,
                 3,  0, -1,  0,  0,  0,
                -2,  1,  0,  0,  0,  0,
                 0, -6,  5,  0, -3,  2,
                 6,  0, -4,  3,  0, -1,
                -5,  4,  0, -2,  1,  0;

        std::cout << output << std::endl;
        std::cout << gt << std::endl;

        REQUIRE((output - gt).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Compounding. 2nd order. Simple case")
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
        
        mrob::SE3Cov new_uncertainty = uncertainty.compound_2nd_order(pose_increment, increment_covariance);

        Mat6 gt_cov(Mat6::Zero());
        gt_cov <<   0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,
                    0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,
                    0.   ,  0.   ,  0.11 ,  0.   , -0.05 ,  0.   ,
                    0.   ,  0.   ,  0.   ,  0.02 ,  0.   ,  0.   ,
                    0.   ,  0.   , -0.05 ,  0.   ,  0.045,  0.   ,
                    0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ;

        REQUIRE((new_uncertainty.cov() - gt_cov).norm() == Approx(0.0).margin(1e-12));

        Mat4 gt_pose(Mat4::Zero());
        gt_pose <<  0.070737201667703 , -0.9974949866040546,  0.                ,  1.1649966577360362,
                    0.9974949866040546,  0.070737201667703 ,  0.                ,  0.6195085322215313,
                    0.                ,  0.                ,  1.                ,  0.                ,
                    0.                ,  0.                ,  0.                ,  1.                ;
        
        REQUIRE((new_uncertainty.T() - gt_pose).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Compounding. 4th order. Simple case")
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

        mrob::SE3Cov new_uncertainty = uncertainty.compound_4th_order(pose_increment, increment_covariance);

        Mat6 gt_cov(Mat6::Zero());

        gt_cov <<   0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000,
                    0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000,
                    0.0000000000, 0.0000000000, 0.1100000000, 0.0000000000, -0.0500000000, 0.0000000000,
                    0.0000000000, 0.0000000000, 0.0000000000, 0.0201541666666667, 0.0000000000, 0.0000000000,
                    0.0000000000, 0.0000000000, -0.0500000000, 0.0000000000, 0.0450500000, 0.0000000000,
                    0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000, 0.0000000000;

        Mat4 gt_pose(Mat4::Zero());
        gt_pose <<  0.070737201667703 , -0.9974949866040546,  0.                ,  1.1649966577360362,
                    0.9974949866040546,  0.070737201667703 ,  0.                ,  0.6195085322215313,
                    0.                ,  0.                ,  1.                ,  0.                ,
                    0.                ,  0.                ,  0.                ,  1.                ;

        REQUIRE((new_uncertainty.T() - gt_pose).norm() ==Approx(0.0).margin(1e-12));
        std::cout << new_uncertainty.cov() << std::endl;
        REQUIRE((new_uncertainty.cov() - gt_cov).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("Compounding. 2nd order. General case")
    {
        Mat61 xi;
        xi << 1,2,-1,0.5,-1,3;
        mrob::SE3 T(xi);
        Mat6 cov(Mat6::Zero());
        cov.diagonal() << 0.01,0.02,0.03,0.01,0.05,1;

        mrob::SE3Cov uncertainty(T, cov);

        xi << -1,0.2,-1.5,-1.0,-10,-4;
        mrob::SE3 pose_increment(xi);

        Mat6 increment_covariance(Mat6::Zero());
        increment_covariance.diagonal() << 0.1,0.1,0.2,0.01,0.01,0.1;
        
        mrob::SE3Cov new_uncertainty = uncertainty.compound_2nd_order(pose_increment, increment_covariance);

        Mat6 gt_cov(Mat6::Zero());
        gt_cov <<    0.1151107300776825, -0.0192270834242289, -0.0107365128807755,  0.0401386236014494,  0.0897149405701326,  0.2322954773132835,
                    -0.0192270834242289,  0.19233423236664  ,  0.0403918472911388, -0.2256272255457387, -0.056782432343284 ,  0.1348639333621249,
                    -0.0107365128807755,  0.0403918472911388,  0.1525550375556777, -0.3245405281697261, -0.1367656231871551,  0.0166438087418345,
                    0.0401386236014494, -0.2256272255457387, -0.3245405281697261,  0.9725717416077854,  0.3536034881338836, -0.1502820245732939,
                    0.0897149405701326, -0.056782432343284 , -0.1367656231871551,  0.3536034881338836,  0.3357308049531596,  0.1922097626238213,
                    0.2322954773132835,  0.1348639333621249,  0.0166438087418345, -0.150282024573294 ,  0.1922097626238213,  1.730000798568441 ;

        REQUIRE((new_uncertainty.cov() - gt_cov).norm() == Approx(0.0).margin(1e-12));

        Mat4 gt_pose(Mat4::Zero());
        gt_pose <<  -0.7079330997016517, -0.6837823830436842,  0.1768399812992293, -0.6167626040635867,
                    -0.7051530118471613,  0.6984259413063296, -0.1223128545707475, -7.846417457327979 ,
                    -0.0398742552242469, -0.2112885636977738, -0.9766100483923171,  8.341550757437583 ,
                     0.                ,  0.                ,  0.                ,  1.                ;

        REQUIRE((new_uncertainty.T() - gt_pose).norm() == Approx(0.0).margin(1e-12));
    }


    SECTION("Compounding. 4th order. General case")
    {
        Mat61 xi;
        xi << 1,2,-1,0.5,-1,3;
        mrob::SE3 T(xi);
        Mat6 cov(Mat6::Zero());
        cov.diagonal() << 0.01,0.02,0.03,0.01,0.05,1;

        mrob::SE3Cov uncertainty(T, cov);

        xi << -1,0.2,-1.5,-1.0,-10,-4;
        mrob::SE3 pose_increment(xi);

        Mat6 increment_covariance(Mat6::Zero());
        increment_covariance.diagonal() << 0.1,0.1,0.2,0.01,0.01,0.1;

        mrob::SE3Cov new_uncertainty = uncertainty.compound_4th_order(pose_increment, increment_covariance);

        Mat6 gt_cov(Mat6::Zero());
        gt_cov <<   0.11564860714102614 , -0.01898674488142607 , -0.0106470419401024  ,  0.039528383578574314,  0.09062102246996999 ,  0.23229267256299657 ,
                    -0.01898674488142607 ,  0.1915211696615563  ,  0.04022354792742576 , -0.22483440388338105 , -0.056250508369940895,  0.13441597165910535 ,
                    -0.010647041940102396,  0.04022354792742576 ,  0.15151142678698273 , -0.3237690146623021  , -0.13630973777653121 ,  0.016702545778983537,
                    0.039528383578574314, -0.22483440388338102 , -0.32376901466230207 ,  1.0144673094022802  ,  0.3530100712508114  , -0.14929818437639442 ,
                    0.09062102246996998 , -0.0562505083699409  , -0.13630973777653121 ,  0.35301007125081146 ,  0.36755707836164986 ,  0.19404132162170293 ,
                    0.23229267256299657 ,  0.13441597165910535 ,  0.016702545778983547, -0.14929818437639444 ,  0.19404132162170293 ,  1.6873818729288543 ;

        REQUIRE((new_uncertainty.cov() - gt_cov).norm() == Approx(0.0).margin(1e-12));

        Mat4 gt_pose(Mat4::Zero());
        gt_pose <<  -0.7079330997016517, -0.6837823830436842,  0.1768399812992293, -0.6167626040635867,
                    -0.7051530118471613,  0.6984259413063296, -0.1223128545707475, -7.846417457327979 ,
                    -0.0398742552242469, -0.2112885636977738, -0.9766100483923171,  8.341550757437583 ,
                     0.                ,  0.                ,  0.                ,  1.                ;

        REQUIRE((new_uncertainty.T() - gt_pose).norm() == Approx(0.0).margin(1e-12));
    }

    SECTION("SE3Cov Compounding. Time benchmark")
    {
        Mat61 xi;
        xi << 1,2,-1,0.5,-1,3;
        mrob::SE3 T(xi);
        Mat6 cov(Mat6::Zero());
        cov.diagonal() << 0.01,0.02,0.03,0.01,0.05,1;

        xi << -1,0.2,-1.5,-1.0,-10,-4;
        mrob::SE3 pose_increment(xi);

        Mat6 increment_covariance(Mat6::Zero());
        increment_covariance.diagonal() << 0.1,0.1,0.2,0.01,0.01,0.1;

        mrob::SE3Cov uncertainty(T, cov);

        int n_cycles = 10000;

        std::cout << "Measuring the time for 2nd order compounding:" << std::endl;
        auto start = std::chrono::system_clock::now();
        for (int i = 0; i < n_cycles; i++)
        {
            uncertainty.compound_2nd_order(pose_increment, increment_covariance);
        }
        auto end = std::chrono::system_clock::now();
        auto elapsed_2nd = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << n_cycles << " cycles : " << elapsed_2nd.count() << "ms" << std::endl;
        std::cout << "1 cycle of 2nd order: " << 1.*elapsed_2nd.count()/n_cycles << "ms" << std::endl;

        std::cout << "Measuring the time for 4th order compounding:" << std::endl;
        start = std::chrono::system_clock::now();
        for (int i = 0; i < n_cycles; i++)
        {
            uncertainty.compound_4th_order(pose_increment, increment_covariance);
        }
        end = std::chrono::system_clock::now();
        auto elapsed_4th = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << n_cycles << " cycles : " << elapsed_4th.count() << "ms" << std::endl;

        std::cout << "1 cycle of 4th order: " << 1.*elapsed_4th.count()/n_cycles << "ms" << std::endl;
    }

    SECTION("mul() operator test")
    {
        Mat61 xi;
        xi << 1,2,-1,0.5,-1,3;
        mrob::SE3 T(xi);
        Mat6 sigma(Mat6::Zero());
        sigma.diagonal() << 0.01,0.02,0.03,0.01,0.05,1;

        mrob::SE3Cov cov(T, sigma);

        xi << -1,0.2,-1.5,-1.0,-10,-4;
        mrob::SE3 pose_increment(xi);

        Mat6 increment_sigma(Mat6::Zero());
        increment_sigma.diagonal() << 0.1,0.1,0.2,0.01,0.01,0.1;

        mrob::SE3Cov increment_cov(pose_increment, increment_sigma);

        REQUIRE((cov.mul(increment_cov).cov() - cov.compound_2nd_order(increment_cov).cov()).norm() == Approx(0.0).margin(1e-12));
    }

    // SECTION("Type cast time benchmark test")
    // {
    //     Mat61 xi;
    //     xi << 1,2,-1,0.5,-1,3;
    //     mrob::SE3 T(xi);
    //     Mat6 cov(Mat6::Zero());
    //     cov.diagonal() << 0.01,0.02,0.03,0.01,0.05,1;

    //     xi << -1,0.2,-1.5,-1.0,-10,-4;
    //     mrob::SE3 pose_increment(xi);

    //     Mat6 increment_covariance(Mat6::Zero());
    //     increment_covariance.diagonal() << 0.1,0.1,0.2,0.01,0.01,0.1;

    //     mrob::SE3Cov uncertainty(T, cov);

    //     mrob::SE3Cov tmp_cov(pose_increment, increment_covariance);
    //     int n_cycles = 10000;

    //     std::cout << "Measuring the time for (const SE3):" << std::endl;
    //     auto start = std::chrono::system_clock::now();
    //     for (int i = 0; i < n_cycles; i++)
    //     {
    //         uncertainty.compound_2nd_order((const SE3)tmp_cov,tmp_cov.cov());
    //     }
    //     auto end = std::chrono::system_clock::now();
    //     auto elapsed_1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //     std::cout << n_cycles << " cycles : " << elapsed_1.count() << "ms" << std::endl;

    //     std::cout << "Measuring the time for static_cast:" << std::endl;
    //     start = std::chrono::system_clock::now();
    //     for (int i = 0; i < n_cycles; i++)
    //     {
    //         uncertainty.compound_2nd_order(static_cast<SE3>(tmp_cov),tmp_cov.cov());
    //     }
    //     end = std::chrono::system_clock::now();
    //     auto elapsed_2 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //     std::cout << n_cycles << " cycles : " << elapsed_2.count() << "ms" << std::endl;

    //     std::cout << "Measuring the time for SE3():" << std::endl;
    //     start = std::chrono::system_clock::now();
    //     for (int i = 0; i < n_cycles; i++)
    //     {
    //         uncertainty.compound_2nd_order(SE3(tmp_cov),tmp_cov.cov());
    //     }
    //     end = std::chrono::system_clock::now();
    //     auto elapsed_3 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //     std::cout << n_cycles << " cycles : " << elapsed_3.count() << "ms" << std::endl;


    //     std::cout << "Measuring the time for compiler cast:" << std::endl;
    //     start = std::chrono::system_clock::now();
    //     for (int i = 0; i < n_cycles; i++)
    //     {
    //         uncertainty.compound_2nd_order(tmp_cov,tmp_cov.cov());
    //     }
    //     end = std::chrono::system_clock::now();
    //     auto elapsed_4 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //     std::cout << n_cycles << " cycles : " << elapsed_4.count() << "ms" << std::endl;
    // }
}
