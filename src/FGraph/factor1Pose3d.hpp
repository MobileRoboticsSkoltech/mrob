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
    int getDim() const {return dim_;};
    /**
     * Evaluates residuals and Jacobians
     */
    void evaluate();
    /**
     * Jacobians are not evaluated, just the residuals
     */
    void evaluateLazy();
    Mat61 getObs() const {return obs_;};
    Mat6 getJacobian(std::shared_ptr<Node> &n) const;
    Mat6 getCovariance() const {return obsCov_;};
    void print() const;

  protected:
    int dim_;//fixed to 6, a RBT
    Mat61 obs_;
    lie::SE3 Tobs_;
    Mat6 obsCov_;
    Mat6 J1_;
    Mat61 r_;// residuals

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}



#endif /* FACTOR1POSE3D_HPP_ */
