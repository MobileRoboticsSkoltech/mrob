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
    uint_t N = 180;
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
            std::cout << "sinc (" << x << ") = " << std::setprecision(40) << res << std::endl;
            std::cout << "tayl (" << x << ") = " << std::setprecision(40) << resTaylor << std::endl;
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
            double resTaylor = 0.5 - x*x/24; // number obtained below *1/2
            //std::cout << "cosc (" << x << ") = " << std::setprecision(40) << res << std::endl;
            //std::cout << "tayl (" << x << ") = " << std::setprecision(40) << resTaylor << std::endl;
            std::cout << "diff (" << x << ") = " << std::setprecision(20) << res - resTaylor << std::endl;
        }
    }
    // Testing Taylor approximation for (1-cos)/x2.
    // - First derivative  = 0 (even function)
    // - Second derivative.  Lets approximate this value from the right
    // RESULT -> -1/12 (0.08333)
    if (0)
    {
        for ( auto x : X)
        {
            double c = std::cos(x), s = std::sin(x), xx = x*x;
            double res = (c*xx - s*x - 3*s*x + 6 - 6*c)/(xx*xx);
            std::cout << "Second derivative of (1-cos)/theta^2 approximated by the right= ("
                      << x << ") = " << std::setprecision(40) << res << std::endl;
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
            double resTaylor = 1 + 12*xx;// + xx*xx*120.0;
            std::cout << "x/sin (" << x << ") = " << std::setprecision(40) << res << std::endl;
            //std::cout << "tayl (" << x << ") = " << std::setprecision(40) << resTaylor << std::endl;
            //std::cout << "diff (" << x << ") = " << std::setprecision(20) << res - resTaylor << std::endl;
        }
    }


    // plot acos around -1
    // --------------------------------------------------------------------------
    // Results:
    // * around 1 ->converges to 0, x < 10-17 is zero and min value = 1.4e-8
    // * around -1, same as above

     if (1)
     {
         for ( auto x : X)
         {
             double xp = -1.0 + x;
             double res = std::acos(xp);
             //double xx = xp*xp;
             std::cout << "acos (" << x << ") = " << std::setprecision(40) << res << std::endl;
             //std::cout << "acos (" << x << ") = " << std::setprecision(40) << M_PI - res << std::endl;
         }
     }


    // plot (x - sin(x))/x^3
    // --------------------------------------------------------------------------
    // Results: checked, converges to 1/6, numerical results inconsistent x < 1e-6
    // Taylor approximation x < 1e-3 already works well (error < 1e-8)
    if (0)
    {
        for ( auto x : X)
        {
            double xx = x*x;
            double res = (x - std::sin(x))/xx/x;
            double resTaylor = 1/6.0 - xx/120;
            //std::cout << "(x-sin)/x^3 (" << x << ") = " << std::setprecision(40) << res << std::endl;
            //std::cout << "tayl (" << x << ") = " << std::setprecision(40) << resTaylor << std::endl;
            std::cout << "diff (" << x << ") = " << std::setprecision(20) << res - resTaylor << std::endl;
        }
    }

    // Result -> -0.0166 (-1/60) for x < e-3 already starts degradating
    if (0)
    {
        for ( auto x : X)
        {
            double c = std::cos(x), s = std::sin(x), xx = x*x;
            double res = (6*x + s*xx + 6*c*x - 12*s )/(xx*xx*x);
            std::cout << "2nd Derivative of (0-sin)/theta^3 approximated by the right= ("
                      << x << ") = " << std::setprecision(40) << res << std::endl;
        }
    }

    // plot 1/x^2(1 - s/(2(1-c)))
    // --------------------------------------------------------------------------
    // Results: checked, converges to 1/12, numerical results inconsistent x < 1e-4. Simplified version (res2) is equally poor
    // Taylor approximation x < 1e-3 already works well (error < 1e-8)
    if (0)
    {
        for ( auto x : X)
        {
            double c = std::cos(x), s = std::sin(x), xx = x*x;
            double res = (1-0.5*s*x/(1-c))/xx;
            double res2 = (1 - c - 0.5*s*x)/xx/(1-c);
            double resTaylor = 1/12.0 + xx/720;
            //std::cout << "f (" << x << ") = " << std::setprecision(40) << res << std::endl;
            //std::cout << "f2(" << x << ") = " << std::setprecision(40) << res2 << std::endl;
            //std::cout << "tayl (" << x << ") = " << std::setprecision(40) << resTaylor << std::endl;
            std::cout << "diff (" << x << ") = " << std::setprecision(20) << res - resTaylor << std::endl;
        }
    }

    // Limit of the derivative. First order 0 (even function)
    //           second derivative -> 1/360
}
