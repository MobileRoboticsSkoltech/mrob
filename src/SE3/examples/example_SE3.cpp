/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * example_SE3.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include "skmr/SE3.hpp"
#include "skmr/SO3.hpp"
#include <iostream>
#include <Eigen/LU>
#include <cmath>


int main()
{

    // SO3 tests
    // ========================================================

    // Testing the Identity element
    {
    skmr::SO3 R;
    R.print();
    std::cout << "Identity element error= " << R.ln_vee().norm() << std::endl;
    }

    // Testing the Identity element plus epsilon
    {
    Mat31 w;
    w << 1e-15, 0, 0;
    skmr::SO3 R(w);
    std::cout << "Identity element error epsilon = " << R.ln_vee().norm() << std::endl;
    }

    // Testing Regular
    {
    Mat31 w;
    w << 1.2, -0.3, 0.2;
    skmr::SO3 R = skmr::SO3(w);
    std::cout << "Exponent and Log test " << (R.ln_vee() - w).norm() << std::endl;
    }

    // Testing Pi
    {
    Mat31 w;
    w << M_PI, 0.0, 0.0;
    skmr::SO3 R = skmr::SO3(w);
    //R.print();
    std::cout << "Pi rotation 1 component = " << (R.ln_vee() - w).norm() << std::endl;
    }

    {
    Mat31 w;
    w << M_PI*std::sqrt(1.0/3), M_PI*std::sqrt(1.0/3), M_PI*std::sqrt(1.0/3);
    skmr::SO3 R = skmr::SO3(w);
    std::cout << "Pi rotation 3 components= " <<  (R.ln_vee() - w).norm() << std::endl;
    }

    // testing operators
    Mat31 w;
    w << M_PI, 0, 0;
    skmr::SO3 R;
    std::cout << "\ntesting operators\n";
    Mat3 w_hat = R.ln();
    std::cout << w_hat << std::endl;
    R.print_lie();

    std::cout << "testing inverse"  << std::endl;
    skmr::SO3 Rt = R.inv();
    std::cout << "invers = " << Rt  << std::endl;

    if (1)
    {
    // SE3 tests
    // ========================================================
    // testing the constructor
    std::cout << "\n\nSE3 tests"  << std::endl;
    skmr::SE3 T1;
    T1.print();
    T1.print_lie();
    Mat61 xi;
    xi << 1,0,-0.2, 5, 10, 2;
    skmr::SE3 T(xi);
    T.print();
    xi << T.ln_vee();
    skmr::SE3 T2(xi);
    T2.print();
    std::cout << "Matrix distance = " << (T-T2).norm() << std::endl;

    std::cout << "testing update\n";
    T2.update(xi);
    T2.print();
    Mat41 v;
    v << 1.0, 3.2, -1.2, 1.0;
    std::cout << T2*v << std::endl;
    v = T2*v;
    std::cout << v << std::endl;

    std::cout << "testing inverse"  << std::endl;
    skmr::SE3 Tt = T2.inv();
    std::cout << "invers = " << Tt  << std::endl;
    std::cout << "Matrix distance = " << (Tt-T2.inverse()).norm() << std::endl;
    Mat61 xi2 = -T2.ln_vee();
    skmr::SE3 T22( xi2);
    std::cout << "Matrix distance by negating= " << (T22-T2.inverse()).norm() << std::endl;

    std::cout << "testing adjoint"  << std::endl;
    std::cout << "Adjoint= " << Tt.adj()  << std::endl;

    }

}
