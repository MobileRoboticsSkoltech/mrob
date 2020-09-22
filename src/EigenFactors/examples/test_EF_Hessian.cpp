/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * See LICENSE file in the root of the mrob library.
 *
 *
 * test_EF_Hessian.cpp
 *
 *  Created on: Aug 15, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include <iostream>

#include "mrob/SE3.hpp"


using namespace mrob;

// This test will output the numerical derivatives and the calculated derivative
int main()
{
    // create transformation
    Mat61 x = Mat61::Zero();//Approximations only work well around the Identity, thus x = 0
    SE3 T(x);
    //std::cout << "Initial random transformation = \n" << x << std::endl;
    //T.print();


    // gnerative basis in the manifold
    Mat6 G = Mat6::Identity();

    //std::cout << "Generative basis\n" << G1 << G2 << G3 << G4 << G5 << G6 << std::endl;

    // calculate numerical derivatives d Exp / Dxi = d/dxi (Exp(x+e*Gi)-Exp(x) )/ eps,
    //    where Gi are the generative basis, and x usually is zero (transformation around the Identy)
    double eps = 1e-4;
    if (0)
    {
        T = SE3(x);
        Mat61 dx = x + eps*G.col(2);
        SE3 Teps = SE3(dx);
        std::cout << "Initial random transformation = \n" << std::endl;
        T.print_lie();
        std::cout << "dt lie algebra params = \n" << std::endl;
        Teps.print_lie();
        std::cout << (Teps.T() - T.T())/eps << std::endl;
    }


    // Numerical derivates not around the identity
    // dT/dx = Gi + 0.5
    if (1)
    {
        std::cout << "\n\n Derivatives around a random element \n" << std::endl;
        x = Mat61::Random() * 0.1;
        T = SE3(x);
        uint_t i = 1;
        Mat61 dx = x+eps*G.col(i);
        SE3 Teps = SE3(dx);
        //std::cout << "Gi hat x\n"<< hat6(G.col(i)) * hat6(x) << "\n and x^ Gi \n"<< hat6(x) * hat6(G.col(i)) << std::endl;
        Mat4 derivative = (Teps.T() - T.T())/eps;
        Mat4 Gi = hat6(G.col(i)) , xhat = hat6(x);
        Mat4 analytical = Gi + 0.5*Gi * xhat + 0.5 * xhat * Gi;
        std::cout << derivative << "\n vs analytical =\n" << analytical << std::endl;
    }



    // calculate numerical second order derivatives
    if (1)
    {
        std::cout << "\n\n Second order derivatives around a the identity\n" << std::endl;
        //x = Mat61::Random() * 0.1;
        x = Mat61::Zero();
        T = SE3(x);
        uint_t i = 0, j = 2;
        Mat61 dxi = x + eps*G.col(i);
        Mat61 dxj = x + eps*G.col(j);
        Mat61 dxij = x + eps*G.col(i) + eps*G.col(j);
        SE3 Ti(dxi), Tj(dxj), Tij(dxij);
        Mat4 derivative = (Tij.T() - Ti.T() - Tj.T() + T.T() )/eps/eps;
        Mat4 Gi = hat6(G.col(i)), Gj = hat6(G.col(j));
        Mat4 analytical = 0.5*Gi * Gj + 0.5 * Gj * Gi;
        std::cout << derivative << "\n vs analytical =\n" << analytical << std::endl;

    }

}
