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
 * matrix.
 */

class DenseGaussNewton
{
public:
    DenseGaussNewton(std::shared_ptr<FGraph> fg);
    virtual ~DenseGaussNewton();
    void buildProblem();
    void solve();
protected:
    std::shared_ptr<FGraph> fg_;// reference to the graph structure
    MatX Information_;

};

}


#endif /* SOLVERDENSE_HPP_ */
