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
 * optimizer.cpp
 *
 *  Created on: Jul 8, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/optimizer.hpp"
#include <Eigen/LU> // for inverse and determinant
#include <iostream>

using namespace mrob;

Optimizer::Optimizer(matData_t solutionTolerance, matData_t lambda) :
        solutionTolerance_(solutionTolerance), lambda_(lambda)
{

}

Optimizer::~Optimizer()
{

}


 uint_t Optimizer::optimize(optimMethod method, double lambda)
{
    optimization_method_ = method;
    switch(method)
    {
      case NEWTON_RAPHSON:
          return optimize_newton_raphson();
      case LEVENBERG_MARQUARDT_SPHER:
      case LEVENBERG_MARQUARDT_ELLIP:
          lambda_ = lambda;
          return optimize_levenberg_marquardt();
    }
    return 0;
}


uint_t Optimizer::optimize_newton_raphson_one_iteration(bool useLambda)
{
    // 1) build problem: Gradient and Hessian and re-evaluates
    calculate_gradient_hessian();
    if (useLambda)
    {
        if (optimization_method_ == LEVENBERG_MARQUARDT_SPHER)
        {
            for (uint_t i = 0; i < hessian_.diagonalSize() ; ++i)
                hessian_(i,i) += lambda_;
        }
        if (optimization_method_ == LEVENBERG_MARQUARDT_ELLIP)
        {
            for (uint_t i = 0; i < hessian_.diagonalSize() ; ++i)
                hessian_(i,i) *= 1.0 + lambda_;
        }
    }

    // 2) dx = - h^-1 * grad XXX test for singularities?
    dx_ = - hessian_.inverse() * gradient_;
    // 3) update the solution
    this->update_state(dx_);

    return 1;
}

uint_t Optimizer::optimize_newton_raphson()
{
    uint_t iters = 0;
    // Calculate error also estimates planes, which are necessary for gradients. XXX this can cause bugs on the first iteration
    matData_t previous_error = this->calculate_error(), diff_error;
    do
    {
        this->optimize_newton_raphson_one_iteration(false);
        matData_t current_error = this->calculate_error();
        //std::cout << "iter " << iters << ", error = " << current_error <<std::endl;
        diff_error = previous_error - current_error;
        previous_error = current_error;
        iters++;
    }while(fabs(diff_error) > solutionTolerance_ && iters < 1e2);


    return iters;
}

uint_t Optimizer::optimize_levenberg_marquardt()
{
    // LM trust region as described in Bertsekas (p.105)
    // sigma reference to the fidelity of the model at the proposed solution \in [0,1]
    matData_t sigma1(0.25), sigma2(0.8);// 0 < sigma1 < sigma2 < 1
    matData_t beta1(2.0), beta2(0.25); // lambda updates multiplier values, beta1 > 1 > beta2 >0
    uint_t iters = 0;
    matData_t previous_error = calculate_error(), diff_error, current_error;
    bool improvement; // variable for controlling when no update is done and number of iterations is exceeded.
    do
    {
        iters++;
        // 1) solve the current subproblem by Newton Raphson
        this->bookkeep_state();
        optimize_newton_raphson_one_iteration(true);
        current_error = calculate_error();
        //std::cout << "iter " << iters << ", error = " << current_error << ", lambda = "<< lambda_ << std::endl;
        diff_error = previous_error - current_error;
        improvement = true;

        //TODO there is an error here, diff error is correct, but the current state get changed! how

        // 2) Check for convergence, hillclimb
        if (diff_error < 0)
        {
            //std::cout << "no improvement\n";
            lambda_ *= beta1;
            this->update_state_from_bookkeep();
            improvement = false;
            continue;
        }
        previous_error = current_error;

        // 2.1) check for convergence, terminal
        if (diff_error < solutionTolerance_)
            return iters;

        // 3 Fidelity of the quadratized model vs non-linear error evaluation.
        // f = err(x_k) - err(x_k + dx)  ( >0 if upgrade)
        //     err(x_k) - m_k(dx)
        // where m_k is the quadratized model m_k(dx) = err(x_k) + dx'*Grad r + 0.5 dx'(Hessian + LM)dx
        // => f = d err / (-dx'*Grad r - 0.5 dx'(Hessian + LM)dx)
        matData_t modelFidelity = diff_error / (-dx_.dot(gradient_) - 0.5*dx_.dot(hessian_* dx_));


        // 4) update lambda
        if (modelFidelity < sigma1)
            lambda_ *= beta1;
        if (modelFidelity > sigma2)
            lambda_ *= beta2;

    }while(iters < 1e2);

    if (!improvement)
    {
        this->update_state_from_bookkeep();//If no improvement shown, undo again
    }


    // output
    std::cout << "Optimizer::optimize_levenberg_marquardt: failed to converge after "
              << iters << " iterations and error " << calculate_error()
              << std::endl;

    return iters;
}
