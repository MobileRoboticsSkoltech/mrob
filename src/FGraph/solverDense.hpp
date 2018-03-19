/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * solverDense.hpp
 *
 *  Created on: Feb 27, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef SOLVERDENSE_HPP_
#define SOLVERDENSE_HPP_

#include "FGraph.hpp"

namespace fg{

/**
 * The SolverDenseGaussNewton class is meant for a reduced dimensionality
 * in state variables (nodes) and multiple observations affecting them (factors).
 *
 * Given the nature of the problem, we have a reduced but dense information
 * matrix. No optimization is carried out for ordering or storing sparse matrices.
 */

class DenseGaussNewton
{
public:
    DenseGaussNewton(std::shared_ptr<FGraph> fg);
    virtual ~DenseGaussNewton();
    void solveOnce();
    void solveIncremental();//TODO
    int solve();
    /**
     *  Linearizes the Jacobian and calculates the residuals
     */
    void evaluate();
    /**
     *  Evaluates the residuals on every factor
     */
    void evaluateResiduals();
    /**
     *  Evaluates the Jacobians on every factor
     */
    void evaluateJacobians();
protected:
    std::shared_ptr<FGraph> fg_;// reference to the graph structure
    //exploting symetry of A = Adj'*Adj, we store on a lower triangular matrix
    // more at : https://eigen.tuxfamily.org/dox-devel/group__QuickRefPage.html
    //triangularView<SelfAdjoint>()
    MatX A_;
    MatX1 b_;

};

}


#endif /* SOLVERDENSE_HPP_ */
