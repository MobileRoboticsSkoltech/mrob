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

#include "skmr/FGraph.hpp"

namespace skmr{

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
    // XXX this functions should be protected, or public?
    /**
     *  Linearizes the Jacobians, creates the information matrix and calculates the residuals
     */
    void evaluate();
    /**
     *  Evaluates *only* the residuals on every factor and calculates the Chi2 error
     */
    matData_t evaluateChi2();
    /**
     * Builds the matrices, after evaluating all factors
     */
    void buildMatrices();
protected:
    std::shared_ptr<FGraph> fg_;// reference to the graph structure
    //exploting symetry of I = A'*A, we store on a lower triangular matrix
    // more at : https://eigen.tuxfamily.org/dox-devel/group__QuickRefPage.html
    //triangularView<SelfAdjoint>()
    MatX Information_;
    MatX1 b_;
    bool isProblemEvaluated;

};

}


#endif /* SOLVERDENSE_HPP_ */
