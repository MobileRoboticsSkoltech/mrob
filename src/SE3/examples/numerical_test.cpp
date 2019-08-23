/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * numerical_test.cpp
 *
 *  Created on: Aug 22, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include <iostream>
#include <iomanip>      // std::setprecision
#include <cmath>
#include "mrob/SE3.hpp"
#include "mrob/SO3.hpp"
#include <vector>


int main()
{
    // Test the numerical stability of the sinc(x) function and it's power series.
    uint_t N = 50;
    std::vector<double> X(N);
    X[0] = 1; // initial seed

    // plot results
    for ( uint_t i = 1; i < N ; ++i)
        X[i] = X[i-1]*0.7;


    // plot sinc(x)
    // --------------------------------------------------------------------------
    // Results: function very stable, no difference at all even with the first Taylor exp
    //         second order terms matter for x < 1e-2
    if (0)
    {
        for ( auto x : X)
        {
            double res = std::sin(x)/x;
            double xx = x*x;
            double resTaylor = 1 - xx/6;// + xx*xx/120.0;
            //std::cout << "sinc (" << x << ") = " << std::setprecision(40) << res << std::endl;
            //std::cout << "tayl (" << x << ") = " << std::setprecision(40) << resTaylor << std::endl;
            std::cout << "diff (" << x << ") = " << std::setprecision(20) << res - resTaylor << std::endl;
        }
    }

    // plot (1-cos) / x2
    // --------------------------------------------------------------------------
    // Restuls: cosc very numerically unstable, closer to 1e-8 completely unusable
    /* diff (8.5070591730234808802e-05) = 6.4118925724443442959e-09
    diff (6.8056473384187847041e-05) = -9.1461347384580449216e-09
    diff (1.1417981541647707935e-05) = 3.0173546672340734176e-07
    diff (2.394524282602959026e-06) = -9.6578733299601537965e-06
    diff (4.0173451106474909443e-07) = 0.00011094325262006599075
    diff (8.4249833334846106441e-08) = 0.00052077379577158966484
    diff (1.7668470647783922024e-08) = -0.14435860008239076446
    diff (7.2370055773322972058e-09) = -0.5  XXX Totally incorrect here!!
     */
    if (0)
    {
        for ( auto x : X)
        {
            double xx = x*x;
            double res = (1.0-std::cos(x))/xx;
            double resTaylor = 0.5 - x*x/12;
            //std::cout << "cosc (" << x << ") = " << std::setprecision(40) << res << std::endl;
            //std::cout << "tayl (" << x << ") = " << std::setprecision(40) << resTaylor << std::endl;
            std::cout << "diff (" << x << ") = " << std::setprecision(20) << res - resTaylor << std::endl;
        }
    }


    // plot x /sin(x) around 1 and -1
    // --------------------------------------------------------------------------
    // Results:
    //   * Around 0 : slightly less stability than sinc, but still pretty fine.
    //                er(1e-2) = e-4   , er(1e-5) = e-10
    //   * around pi :
    //   * arnd  -pi :

     if (0)
    {
        for ( auto x : X)
        {
            double xp = M_PI - x;
            double res = xp / std::sin(xp);
            double xx = xp*xp;
            double resTaylor = 1 + 6*xx;// + xx*xx*120.0;
            std::cout << "x/sin (" << x << ") = " << std::setprecision(40) << res << std::endl;
            //std::cout << "tayl (" << x << ") = " << std::setprecision(40) << resTaylor << std::endl;
            //std::cout << "diff (" << x << ") = " << std::setprecision(20) << res - resTaylor << std::endl;
        }
    }


    // plot acos
    // --------------------------------------------------------------------------
    // Results: Stable function but if input is not in [-1,1] outputs nan => prepare this case in Ln
    //    if not, this function is (of course, this is cmath) very stable.
    if (1)
    {
        for ( auto x : X)
        {
            double epsPi = -1.0+x;
            double res = std::acos(epsPi);
            double resTaylor = M_PI - 1.0/std::sqrt(1.0-epsPi*epsPi) * (x);
            if (res != res)
                std::cout << "nana here\n";
            std::cout << "acos (" << 1.0-x << ") = " << std::setprecision(40) << res << std::endl;
            //std::cout << "tayl (" << x << ") = " << std::setprecision(40) << resTaylor << std::endl;
            //std::cout << "diff (" << epsPi << ") = " << std::setprecision(40) << res - resTaylor << std::endl;
        }
    }


    // plot 0.5 *x / sin
    // --------------------------------------------------------------------------
    // Results
    if (0)
    {
        for ( auto x : X)
        {
            double res = 0.5 * x / std::sin(x);
            double xx = x*x;
            double resTaylor = 0.5 + xx/12;// + xx*xx*120.0;
            //std::cout << "x/sin (" << x << ") = " << std::setprecision(40) << res << std::endl;
            //std::cout << "tayl (" << x << ") = " << std::setprecision(40) << resTaylor << std::endl;
            std::cout << "diff (" << x << ") = " << std::setprecision(20) << res - resTaylor << std::endl;
        }
    }
}
