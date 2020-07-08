/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * optimizer.cpp
 *
 *  Created on: Jul 8, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/optimizer.hpp"

using namespace mrob;

Optimizer::Optimizer(matData_t solutionTolerance, matData_t lambda) :
        solutionTolerance_(solutionTolerance), lambda_(lambda)
{

}

Optimizer::~Optimizer()
{

}


 Optimizer::optimize(optimMethod method)
{
    switch(method)
    {
      case RN:
          optimize_newton_raphson(false);
          break;
      case LM-S:
      case LM-E:
          optmize_levenberg_marquard();
          break;
    }
}


void Optimizer::optimize_newton_raphson(bool useLambda)
{

}
