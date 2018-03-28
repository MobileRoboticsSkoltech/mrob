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


#include "SE3.hpp"
#include "SO3.hpp"
#include <iostream>
#include <Eigen/LU>


int main()
{

    // SO3 tests
    // ========================================================
    Mat31 w;
    w << 0.15, -0.2, 0.1;
    lie::SO3 R = lie::SO3(w);
    //R.update(w);

    // testing operators
    std::cout << "testing operators\n";
    Mat3 w_hat = R.ln();
    std::cout << w_hat << std::endl;
    R.print_lie();

    std::cout << "testing inverse"  << std::endl;
    lie::SO3 Rt = R.inv();
    std::cout << "invers = " << Rt  << std::endl;

    if (1)
    {
    // SE3 tests
    // ========================================================
    // testing the constructor
    std::cout << "SE3 tests"  << std::endl;
    lie::SE3 T1;
    T1.print();
    T1.print_lie();
    Mat61 xi;
    xi << 1,0,-0.2, 5, 10, 2;
    lie::SE3 T(xi);
    T.print();
    xi << T.ln_vee();
    lie::SE3 T2(xi);
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
    lie::SE3 Tt = T2.inv();
    std::cout << "invers = " << Tt  << std::endl;
    std::cout << "Matrix distance = " << (Tt-T2.inverse()).norm() << std::endl;

    }

}
