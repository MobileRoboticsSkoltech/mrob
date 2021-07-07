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
 *  - Newton-Raphson (NR), this is a second order method:
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
	 *  - NR: Newton-Raphson (Gauss_newtow is a variant with approximations in the Hessian)
	 *  - LM_S: Levenberg Marquardt: Spherical
	 *  - LM_E: Levenberg Marquardt: Eliptical
	 */
	enum optimMethod{NEWTON_RAPHSON=0, LEVENBERG_MARQUARDT_SPHER, LEVENBERG_MARQUARDT_ELLIP};
    Optimizer(matData_t solutionTolerance = 1e-4, matData_t lambda = 1e-5);
    virtual ~Optimizer();

    /**
     * Optimization call.
     * Input: optmization method from {NR=0, LM_S, LM_E}
     *       - max_iterations
     *       - lambda: initial value of lambda for LM methods
     * output: number of iterations
     */
    uint_t solve(optimMethod method, uint_t max_iters = 1e2, double lambda = 1e-5);

    /**
     * General abstract functions to implement:
     * Calculate error calculates the current error function
     */
    virtual matData_t calculate_error() = 0;
    /**
     * Gradient calculates the gradient and Hessian
     * This function may be called after calculate_error()
     * or not (some cases of LM). For a general purpose it is required
     * that this function is either:
     * 1) self-contained. No assumptions made and recalculated (redundantly) residuals
     * 2) Most of the times will be called after calculate_error. No recalculations are requires except
     *    inside the update_bookkeep_state which will have invalid residuals and need update (less prefered option)
     */
    virtual void calculate_gradient_hessian() = 0;
    /**
     * Updates the current solution
     */
    virtual void update_state() = 0;
    /**
     * For Levenberg-Marquard
     * This function bookkeeps the current state values
     * This is in case the optimization step does not improve
     */
    virtual void bookkeep_state() = 0;
    /**
     * For Levenberg-Marquard
     * Undoes an incorrect update
     */
    virtual void update_state_from_bookkeep() = 0;

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
    virtual uint_t optimize_newton_raphson_one_iteration(bool useLambda = false) = 0;

    /**
     * Levenberg-Marquardt method, inside will distinguish between elliptic and spherical
     *
     * Input useLambda (default false) builds the NR problem with lambda factor on the diagonal
     *    H' = H + lambda * D. where D depends on which LM has been selected (TODO better)
     */
    uint_t optimize_levenberg_marquardt();

    /**
     * calculate_model_fidely returns the difference between the quadratized
     * model and the current error, as required for the LM algorithm.
     *
     * This is a an abstrct class since the hessian matrix might be different
     * for the sparse and the dense case
     */
    virtual matData_t calculate_model_fidelity(matData_t diff_error) = 0;


    optimMethod optimization_method_ {};
    matData_t solutionTolerance_;
    uint_t max_iters_;
    MatX1 gradient_, dx_;

    // Necessary for LM, Other LM parameters are set to default (see .cpp)
    matData_t lambda_;

};

class OptimizerDense : public Optimizer
{
  public:
    OptimizerDense(matData_t solutionTolerance = 1e-4, matData_t lambda = 1e-5);
    virtual ~OptimizerDense();
  protected:
    uint_t optimize_newton_raphson_one_iteration(bool useLambda = false);
    matData_t calculate_model_fidelity(matData_t diff_error);
    MatX hessian_;
};


class OptimizerSparse: public Optimizer
{
  protected:
    MatX hessian_;
};


}
#endif /* SRC_COMMON_MROB_OPTIMIZER_HPP_ */
