/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
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
    MatX X = MatX::Random(3,N);
    std::cout << "X data: \n" << X << std::endl;

    Mat61 xi = Mat61::Random();
    //Mat61 xi = Mat61::Zero();
    mrob::SE3 T(xi);
    std::cout << "T random transformation: \n" << std::endl;
    T.print();


    MatX Y = T.transformArray(X);
    std::cout << "Y data: \n" << Y << std::endl;

    mrob::SE3 T_arun;
    mrob::PCRegistration::Arun(X,Y,T_arun);
    std::cout << "T solved by Arun method: \n" << std::endl;
    T_arun.print();

    // Solve for GICP
    MatX covX(3,3*N);
    MatX covY(3,3*N);
    for (uint_t i = 0; i < N ; ++i)
    {
        covX.block<3,3>(0,3*i) = Mat3::Identity();
        covY.block<3,3>(0,3*i) = Mat3::Identity();
    }
    mrob::SE3 T_gicp;
    mrob::PCRegistration::Gicp(X,Y,covX,covY,T_gicp);
    mrob::PCRegistration::Gicp(X,Y,covX,covY,T_gicp);
    mrob::PCRegistration::Gicp(X,Y,covX,covY,T_gicp);
    std::cout << "T solved by GICP method: \n" << std::endl;
    T_gicp.print();



    //align_mth::Csample_uniform_SE3 s(0.3, 4);
    //std::cout << "Csample = " << s.sample() << std::endl;




    return 1;
}

