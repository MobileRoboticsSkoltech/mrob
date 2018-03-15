/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * factor2Poses3d.hpp
 *
 *  Created on: Feb 28, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR2POSES3D_HPP_
#define FACTOR2POSES3D_HPP_


#include "factor.hpp"
#include <Eigen/Dense>
#include "matrixBase.hpp"
#include "SE3.hpp" //requires including and linking SE3 library

namespace fg{

/**
 * The Factor2Poses3d is a vertex representing the distribution between
 * two nodePose3d, that is, it is expressing a Rigid Body Transformation
 * between two poses.
 *
 * The state is an observer RBT, and as we have commented, we need to specify
 * the two Nodes that the factor is connecting, which are provided by their
 * shared_ptr's which are at the same time their keys for storage on the FGraph
 *
 * In particular, the residual of this factor is: TODO better formulate
 *   r = ln(T2) - ln(T1*Tobs) = ln(T1^-1*T2) - ln(Tobs)
 */

class Factor2Poses3d : public Factor
{
  public:
    Factor2Poses3d(const Mat61 &observation, std::shared_ptr<Node> &n1,
            std::shared_ptr<Node> &n2, const Mat6 &obsCov);
    ~Factor2Poses3d();
    /**
     * Evaluates residuals and Jacobians
     */
    void evaluate();
    /**
     * Jacobians are not evaluated, just the residuals
     */
    matData_t evaluateError();
    void print() const;

  protected:
    lie::SE3 Tobs_;
    // The Jacobians' correspondant nodes are ordered on the vector<Node>
    // being [0]->J1 and [1]->J2


  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FACTOR2POSES3D_HPP_ */
