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
 * example_SE3.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include <iostream>
#include <Eigen/LU> // for inverse and determinant
#include <cmath>
#include "mrob/SE3.hpp"
#include "mrob/SO3.hpp"


int main()
{


    // TODO please write me as a test unit!!!
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

    //testing bool function isSO3
    {
    Mat31 w;
    w << 1.2, -0.3, 0.2;
    mrob::SO3 R(w);
    std::cout << "Matris is SO3: " << mrob::isSO3(R.R()) << std::endl;
    Mat3 notR = Mat3::Random();
    std::cout << "Matris is SO3: " << mrob::isSO3(notR) << std::endl;
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
    std::cout << "invers = " << Rt.R()  << std::endl;
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
    std::cout << "random element plus epsilon= " << (T.ln_vee() - xi).norm() << std::endl;
    }
    {
    Mat61 xi;
    xi << 1,0,-0.2, 5, 10, 2;
    mrob::SE3 T(xi);
    //T.print();
    std::cout << "Some normal element error = " << (T.ln_vee() - xi).norm() << std::endl;
    }

    //testing bool function isSE3
    {
    Mat61 xi;
    xi << 1,0,-0.2, 5, 10, 2;
    mrob::SE3 T(xi);
    std::cout << "input SE3 and Matris is SE3?: " << mrob::isSE3(T.T()) << std::endl;
    Mat4 notT = Mat4::Random();
    std::cout << "Random Matris is SE3: " << mrob::isSE3(notT) << std::endl;
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
    std::cout << "Matrix distance = " << (T.T()-T2.T()).norm() << std::endl;

    std::cout << "testing update\n";
    T2.update_lhs(xi);
    T2.print();
    Mat41 v;
    v << 1.0, 3.2, -1.2, 1.0;
    std::cout << T2.T()*v << std::endl;
    v = T2.T()*v;
    std::cout << v << std::endl;

    std::cout << "testing inverse"  << std::endl;
    mrob::SE3 Tt = T2.inv();
    std::cout << "invers = " << Tt.T()  << std::endl;
    std::cout << "Matrix distance = " << (Tt.T()-T2.T().inverse()).norm() << std::endl;
    Mat61 xi2 = -T2.ln_vee();
    mrob::SE3 T22( xi2);
    std::cout << "Matrix distance by negating= " << (T22.T()-T2.T().inverse()).norm() << std::endl;

    std::cout << "testing adjoint"  << std::endl;
    std::cout << "Adjoint= " << Tt.adj()  << std::endl;


    // testing subblock matrices
    std::cout << "Subblock methods an matrix:"  << std::endl;
    T.print();
    std::cout << "\nT = " << T.T()  << std::endl;
    std::cout << "\nR = " << T.R()  << std::endl;
    std::cout << "\nt = " << T.t()  << std::endl;

    // Acess inner matrix
    T.ref2T() << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 5, 0,
             0, 0, 0 , 1;
    T.print();


    // testing transformation
    {
    Mat61 xi;
    xi << 1,0,-0.2, 5, 10, 2;
    mrob::SE3 T(xi);
    Mat31 p,p2;
    p << 2,3,-1;
    p2 = p;
    p = T.transform(p);
    p = T.inv().transform(p);
    std::cout << "Testing transformations = " << (p-p2).norm() << std::endl;
    }
    //testing SE3 multiplications



}
