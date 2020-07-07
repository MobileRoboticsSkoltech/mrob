/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * optimizer.hpp
 *
 *  Created on: Jul 4, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef OPTIMIZER_HPP_
#define OPTIMIZER_HPP_

/**
 * The class optimizer provides a level of abstraction for solving
 * second order optimization problems of the form:
 *
 * min_x C(X)
 *
 * The current methods implemented are Newton-Raphson (NR) Gauss-Newton (GN) and
 * Levenberg-Marquard (LM)
 *  given the following requirements:
 *  - C(x): a Cost function
 */


class Optimizer
{
public:
    Optimizer();
    ~Optimizer();

    // TODO how to sperate this optmizers or do a generic class?
    virtual void calculate_error() = 0;
    virtual void calculate_gradient() = 0;
    virtual void calculate_hessian() = 0;
    virtual void update() = 0;

    /**
     * Specific methods for Levenberg-Marquard
     */


};


#endif /* SRC_COMMON_MROB_OPTIMIZER_HPP_ */
