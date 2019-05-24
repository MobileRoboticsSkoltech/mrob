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


#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/factor.hpp"

namespace mrob{

/**
 * The Factor2Poses3d is a vertex representing the distribution between
 * two nodePose3d, that is, it is expressing a Rigid Body Transformation
 * between two poses.
 *
 * The state is an observer RBT, and as we have commented, we need to specify
 * the two Nodes that the factor is connecting, which are provided by their
 * shared_ptr's.
 * We provide the node's Id to get the correspondent Jacobian
 *
 *
 * In particular, the residual of this factor is: TODO better formulate
 *   r = ln(T2) - ln(T1*Tobs) = ln(T1^-1*T2) - ln(Tobs)
 *
 * Constructor functions will be overloaded to include the pointers of the nodes,
 * The convention is from node origin, we observe node destination,
 * such that: Factor2Poses3d(nodeOrigin, nodeTarget, ...
 *
 * The observations relate a pair of nodes. The order matters, since this will
 * affect the order on the Jacobian block matrix
 */

class Factor2Poses3d : public Factor
{
  public:
    Factor2Poses3d(const Mat61 &observation, std::shared_ptr<Node> &nodeOrigin,
            std::shared_ptr<Node> &nodeTarget, const Mat6 &obsInf, bool updateNodeTarget=false);
    ~Factor2Poses3d();
    /**
     * Jacobians are not evaluated, just the residuals
     */
    void evaluate_residuals() override;
    /**
     * Evaluates residuals and Jacobians
     */
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const;

    const Eigen::Ref<const MatX1> get_obs() const {return obs_;};
    const Eigen::Ref<const MatX1> get_residual() const {return r_;};
    const Eigen::Ref<const MatX> get_information_matrix() const {return W_;};
    const Eigen::Ref<const MatX> get_trans_sqrt_information_matrix() const{return WT2_;};
    const Eigen::Ref<const MatX> get_jacobian() const {return J_;};

  protected:
    // The Jacobians' correspondant nodes are ordered on the vector<Node>
    // being [0]->J_origin and [1]->J_target
    // declared here but initialized on child classes
    Mat61 obs_, r_; //and residuals
    SE3 Tobs_; // Transformation from observation. NOTE: In Xorigin frame
    SE3 Tr_; // Residual Transformation
    Mat6 W_;//inverse of observation covariance (information matrix)
    Mat6 WT2_;//transpose and squared root of W.
    Mat<6,12> J_;//Joint Jacobian


  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FACTOR2POSES3D_HPP_ */
