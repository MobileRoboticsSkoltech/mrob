/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * solverDenseGaussNewton.hpp
 *
 *  Created on: Feb 27, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef SOLVERDENSEGAUSSNEWTON_HPP_
#define SOLVERDENSEGAUSSNEWTON_HPP_


namespace skmr{

/**
 * The SolverDenseGaussNewton class is meant for a reduced dimensionality
 * in state variables (nodes) and multiple observations affecting them (factors).
 *
 * Given the nature of the problem, we have a reduced but dense information
 * matrix.
 */

class SolverDenseGaussNewton: public FGraph
{
    SolverDenseGaussNewton();
    virtual ~SolverDenseGaussNewton();
};

}


#endif /* SOLVERDENSEGAUSSNEWTON_HPP_ */
