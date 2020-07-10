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
 *         Note that Gauss-Newton is an instance of this, but approximating the
 *         Hessian by J'*J
 *  - Levenberg-Marquardt: Spherical approximation
 *         x' = x - (H + lambda * I)^-1 * gradient
 *  - Levenberg-Marquardt: Elliptical approximation
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
    Optimizer(matData_t solutionTolerance = 1e-2, matData_t lambda = 1e-6);
    virtual ~Optimizer();

    /**
     * Optimization call.
     * Input: optmization method from {NR=0, LM_S, LM_E}
     * output: number of iterations
     */
    uint_t optimize(optimMethod method);

    /**
     * General abstract functions to implement:
     * Calculate error calculates the current error function
     */
    virtual matData_t calculate_error() = 0;
    /**
     * Gradient calculates the gradient and Hessian
     * This function will be called always after a calculate
     * error, so the method using optimizer can keep some information
     * and no need to re-calculated everything
     */
    virtual void calculate_gradient_hessian() = 0;
    /**
     * Updates the current solution
     */
    virtual void update_state(const MatX1 &dx) = 0;
    /**
     * For Levenberg-Marquard
     * This function bookeeps the current state values
     * This is in case the optimization step does not improve
     */
    virtual void bookeep_states() = 0;
    /**
     * For Levenberg-Marquard
     * Undoes an incorrect update
     */
    virtual void update_bookept_states() = 0;

protected:

    /**
     * Optimizes according to the Newton-Raphson algorithm
     * (second order method).
     *
     *
     * Input useLambda (default false) builds the NR problem with lambda factor on the diagonal
     *    H' = H + lambda * D. where D depends on which LM has been selected (TODO better)
     */
    uint_t optimize_newton_raphson();

    /**
     * One iteration of the RN method
     */
    uint_t optimize_newton_raphson_one_iteration(bool useLambda = false);

    /**
     * Levenberg-Marquardt method, inside will distinguish between elliptic and spherical
     *
     * Input useLambda (default false) builds the NR problem with lambda factor on the diagonal
     *    H' = H + lambda * D. where D depends on which LM has been selected (TODO better)
     */
    uint_t optimize_levenberg_marquardt();


    optimMethod optimization_method_;
    matData_t solutionTolerance_;
    MatX1 gradient_, dx_;
    MatX hessian_;

    // Necessary for LM, Other LM parameters are set to default (see .cpp)
    matData_t lambda_;

};
}


#endif /* SRC_COMMON_MROB_OPTIMIZER_HPP_ */
