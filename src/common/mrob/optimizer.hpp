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

#include "mrob/matrix_base.hpp"

namespace mrob{

/**
 * The class optimizer provides a level of abstraction for solving
 * second order optimization problems of the form:
 *
 * min C(x)
 *
 * The current methods implemented are:
 *  - Newton-Raphson (NR), this is a second order method of the form:
 *         x' = x - (Hessian)^-1 * Gradient
 *
 *         Note that Gauss-Newtow is an instance of this, but approximating the
 *         Hessian by J'*J
 *  - Levenberg-Marquard: Spherical approximation
 *         x' = x - (H + lambda * I)^-1 * gradient
 *  - Levenberg-Marquard: Elliptical approximation
 *         x' = x - (H + lambda * D)^-1 * gradient, where D = diag(H)
 *
 *  given the following requirements:
 *  - C(x): a Cost function
 */


class Optimizer
{
public:
	/**
	 * This enums optimization methods available:
	 *  - NR: Newton-Raphson (Gauss_newtow is a variant with approximmations in the Hessian)
	 *  - LM_S: Levenberg Marquardt: Spherical
	 *  - LM_E: Levenberg Marquardt: Eliptical
	 */
	enum optimMethod{NR=0, LM_S, LM_E};
    Optimizer();
    virtual ~Optimizer();

    /**
     * Once the gradient and Hessian are generated, it optimizes according to
     * the Newton-Raphson algorithm (second order method).
     *
     *
     * Input useLambda (default false) builds the NR problem with lambda factor on the diagonal
     *    H' = H + lambda * D. where D depends on which LM has been selected (TODO better)
     */
    void optimize_gauss_newton(bool useLambda = false);

    /**
     * General abstract functions to implement
     */
    virtual void calculate_error() = 0;
    virtual void calculate_gradient() = 0;
    virtual void calculate_hessian() = 0;
    virtual void update() = 0;

    /**
     * Specific methods for Levenberg-Marquard
     */

protected:
    optimMethod optimMethod_;
    MatX hessian_;
    MatX1 gradient_, dx_;
    matData_t solutionTolerance_;

    // Necessary for LM
    MatX1 diag_hessian_;
    matData_t lambda_;

};
}


#endif /* SRC_COMMON_MROB_OPTIMIZER_HPP_ */
