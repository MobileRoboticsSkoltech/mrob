/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * factor1Pose3d.hpp
 *
 *  Created on: Mar 5, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR1POSE3D_HPP_
#define FACTOR1POSE3D_HPP_


#include "factor.hpp"
#include <Eigen/Dense>
#include "matrixBase.hpp"
#include "SE3.hpp" //requires including and linking SE3 library

namespace fg{

/**
 * The Factor1Poses3d is a vertex representing the distribution
 * of a nodePose3d, pretty much like an anchoring factor.
 *
 * The state is an observed RBT, coincident with the node state it is connected to.
 * We should provide the node's
 * shared_ptr which are at the same time their keys for storage on the FGraph
 *
 * In particular, the residual of this factor is: TODO better formulate
 *   r = obs-x
 */

class Factor1Pose3d : public Factor
{
  public:
    Factor1Pose3d(const Mat61 &observation, std::shared_ptr<Node> &n1,
             const Mat6 &obsCov);
    ~Factor1Pose3d();
    /**
     * Evaluates residuals and Jacobians
     */
    void evaluate();
    /**
     * Returns the chi2 error and fills the residual vector
     */
    matData_t evaluateError();

  protected:
    lie::SE3 Tobs_;

};


}



#endif /* FACTOR1POSE3D_HPP_ */
