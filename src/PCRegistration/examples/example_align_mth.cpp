/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * example_align_mth.cpp
 *
 *  Created on: Feb 2, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include <iostream>
#include "mrob/pc_registration.hpp"


int main()
{
    // test data structures
    uint_t N = 4;
    MatX X = MatX::Random(N,3);
    std::cout << "X data: \n" << X << std::endl;

    Mat61 xi = Mat61::Random();
    //Mat61 xi = Mat61::Zero();
    mrob::SE3 T(xi);
    std::cout << "T random transformation: \n" << std::endl;
    T.print();


    MatX Y = T.transform_array(X);
    std::cout << "Y data: \n" << Y << std::endl;

    mrob::SE3 T_arun;
    mrob::PCRegistration::arun(X,Y,T_arun);
    std::cout << "T solved by Arun method: \n" << std::endl;
    T_arun.print();

    // Solve for GICP
    MatX covX(3*N,3);
    MatX covY(3*N,3);
    for (uint_t i = 0; i < N ; ++i)
    {
        covX.block<3,3>(3*i,0) = Mat3::Identity();
        covY.block<3,3>(3*i,0) = Mat3::Identity();
    }
    mrob::SE3 T_gicp;
    uint_t iters = mrob::PCRegistration::gicp(X,Y,covX,covY,T_gicp);
    std::cout << "T solved by GICP method on " << iters << " iters: \n" << std::endl;
    T_gicp.print();


    // Solve for weighted point optimization
    MatX1 weight = MatX1::Ones(N);
    mrob::SE3 T_wp;
    iters = mrob::PCRegistration::weighted_point(X,Y,weight,T_wp);//TODO change to work with Nx3 convention
    std::cout << "T solved by Weight point Optimization method on " << iters << " iters: \n" << std::endl;
    T_wp.print();


    //align_mth::Csample_uniform_SE3 s(0.3, 4);
    //std::cout << "Csample = " << s.sample() << std::endl;




    return 1;
}

