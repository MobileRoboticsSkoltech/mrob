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


#include <iostream>
#include <Eigen/LU>
#include <cmath>
#include "mrob/SE3.hpp"
#include "mrob/SO3.hpp"


int main()
{

    // SO3 tests
    // ========================================================

    // Testing the Identity element
    {
    mrob::SO3 R;
    R.print();
    std::cout << "Identity element error= " << R.ln_vee().norm() << std::endl;
    }

    // Testing the Identity element plus epsilon
    {
    Mat31 w;
    w << 1e-15, 0, 0;
    mrob::SO3 R = mrob::SO3(w);
    std::cout << "Identity element error epsilon = " << R.ln_vee().norm() << std::endl;
    }

    // Testing Regular
    {
    Mat31 w;
    w << 1.2, -0.3, 0.2;
    mrob::SO3 R = mrob::SO3(w);
    std::cout << "Exponent and Log test " << (R.ln_vee() - w).norm() << std::endl;
    }

    // Testing Pi
    {
    Mat31 w;
    w << M_PI, 0.0, 0.0;
    mrob::SO3 R = mrob::SO3(w);
    //R.print();
    std::cout << "Pi rotation 1 component = " << (R.ln_vee() - w).norm() << std::endl;
    }

    {
    Mat31 w;
    w << M_PI*std::sqrt(1.0/3), M_PI*std::sqrt(1.0/3), M_PI*std::sqrt(1.0/3);
    mrob::SO3 R = mrob::SO3(w);
    std::cout << "Pi rotation 3 components= " <<  (R.ln_vee() - w).norm() << std::endl;
    }

    // testing operators}
    {
    std::cout << "\ntesting operators\n";
    Mat31 w;
    w << M_PI, 0.0, 0.0;
    mrob::SO3 R = mrob::SO3(w);
    R.print();
    Mat3 w_hat = R.ln();
    std::cout << w_hat << std::endl;

    std::cout << "testing inverse"  << std::endl;
    mrob::SO3 Rt = R.inv();
    std::cout << "invers = " << Rt  << std::endl;
    }


    // SE3 tests
    // ========================================================
    // testing the constructor
    std::cout << "\n\nSE3 tests"  << std::endl;
    {
    mrob::SE3 T;
    //T.print();
    std::cout << "Identity element error= " << T.ln_vee().norm() << std::endl;
    }
    {
    Mat61 xi;
    xi << 1e-9,0,0, 20, 100, 4;
    mrob::SE3 T(xi);
    T.print();
    std::cout << "Identity element plus epsilon= " << (T.ln_vee() - xi).norm() << std::endl;
    }
    {
    Mat61 xi;
    xi << 1,0,-0.2, 5, 10, 2;
    mrob::SE3 T(xi);
    //T.print();
    std::cout << "Some normal element error = " << (T.ln_vee() - xi).norm() << std::endl;
    }
    {
    Mat61 xi;
    xi << M_PI,0,0, 5, 100, 2;
    mrob::SE3 T(xi);
    T.print_lie();
    std::cout << "Pi error plus trans= " << (T.ln_vee()-xi).norm() << std::endl;
    }
    {
    Mat61 xi;
    xi << M_PI*std::sqrt(1.0/3)-1e-4, M_PI*std::sqrt(1.0/3), M_PI*std::sqrt(1.0/3), 5, 1000, 200;
    mrob::SE3 T(xi);
    T.print_lie();
    std::cout << "Pi error 3 comp plus trans= " << (T.ln_vee()-xi).norm() << std::endl;
    // The error ind
    }
    mrob::SE3 T1;
    T1.print();
    T1.print_lie();
    Mat61 xi;
    xi << 1,0,-0.2, 5, 10, 2;
    mrob::SE3 T(xi);
    T.print();
    xi << T.ln_vee();
    mrob::SE3 T2(xi);
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
    mrob::SE3 Tt = T2.inv();
    std::cout << "invers = " << Tt  << std::endl;
    std::cout << "Matrix distance = " << (Tt-T2.inverse()).norm() << std::endl;
    Mat61 xi2 = -T2.ln_vee();
    mrob::SE3 T22( xi2);
    std::cout << "Matrix distance by negating= " << (T22-T2.inverse()).norm() << std::endl;

    std::cout << "testing adjoint"  << std::endl;
    std::cout << "Adjoint= " << Tt.adj()  << std::endl;



}
