/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
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
    int N = 4;
    mrob::MatX X = mrob::MatX::Random(N,3);
    std::cout << "X data: \n" << X << std::endl;

    mrob::Mat61 xi = mrob::Mat61::Random();
    //Mat61 xi = Mat61::Zero();
    mrob::SE3 T(xi);
    std::cout << "T random transformation: \n" << std::endl;
    T.print();


    mrob::MatX Y = T.transform_array(X);
    std::cout << "Y data: \n" << Y << std::endl;

    mrob::SE3 T_arun;
    mrob::PCRegistration::arun(X,Y,T_arun);
    std::cout << "T solved by Arun method: \n" << std::endl;
    T_arun.print();

    // Solve for GICP
    mrob::MatX covX(3*N,3);
    mrob::MatX covY(3*N,3);
    for (int i = 0; i < N ; ++i)
    {
        covX.block<3,3>(3*i,0) = mrob::Mat3::Identity();
        covY.block<3,3>(3*i,0) = mrob::Mat3::Identity();
    }
    mrob::SE3 T_gicp;
    int iters = mrob::PCRegistration::gicp(X,Y,covX,covY,T_gicp);
    std::cout << "T solved by GICP method on " << iters << " iters: \n" << std::endl;
    T_gicp.print();


    // Solve for weighted point optimization
    mrob::MatX1 weight = mrob::MatX1::Ones(N);
    mrob::SE3 T_wp;
    iters = mrob::PCRegistration::weighted_point(X,Y,weight,T_wp);//TODO change to work with Nx3 convention
    std::cout << "T solved by Weight point Optimization method on " << iters << " iters: \n" << std::endl;
    T_wp.print();


    //align_mth::Csample_uniform_SE3 s(0.3, 4);
    //std::cout << "Csample = " << s.sample() << std::endl;




    return 1;
}

