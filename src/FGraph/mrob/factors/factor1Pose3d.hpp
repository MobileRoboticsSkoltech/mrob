/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
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


#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/factor.hpp"

namespace mrob{

/**
 * The Factor1Poses3d is a vertex representing the distribution
 * of a nodePose3d, pretty much like an anchoring factor.
 *
 * The state is an observed RBT, coincident with the node state it is connected to.
 *
 * In particular, the residual of this factor is: TODO better formulate
 *   r = obs-x
 */

class Factor1Pose3d : public Factor
{
  public:
    Factor1Pose3d(const Mat61 &observation, std::shared_ptr<Node> &n1,
             const Mat6 &obsInf);
    ~Factor1Pose3d();
    /**
     * Returns the chi2 error and fills the residual vector
     */
    void evaluate_residuals() override;
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const;

    const Eigen::Ref<const MatX1> get_obs() const {return obs_;};
    const Eigen::Ref<const MatX1> get_residual() const {return r_;};
    const Eigen::Ref<const MatX> get_information_matrix() const {return W_;};
    const Eigen::Ref<const MatX> get_trans_sqrt_information_matrix() const{return WT2_;};
    const Eigen::Ref<const MatX> get_jacobian() const {return J_;};


  protected:
    Mat61 obs_, r_; //and residuals
    SE3 Tobs_, Tr_;//Transformation for the observation and the residual
    Mat6 W_;//inverse of observation covariance (information matrix)
    Mat6 WT2_;//transpose and squared root of W. We could delete this variable...
    Mat6 J_;//Jacobian


};


}



#endif /* FACTOR1POSE3D_HPP_ */
