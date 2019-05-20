/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * nodePose3d.hpp
 *
 *  Created on: Feb 28, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef NODEPOSE3D_HPP_
#define NODEPOSE3D_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp" //requires including and linking SE3 library
#include "mrob/node.hpp"

namespace mrob{

class NodePose3d : public Node
{
  public:
    /**
     * For initialization, requires an initial estimation of the state.
     */
    NodePose3d(const Mat61 &initial_x);
    /**
     * Initialization directly on SE3
     */
    NodePose3d(const Mat4 &initial_T);
    virtual ~NodePose3d();
    /**
     * Left update operation corresponds to
     * T'=exp(dxi^)*T
     * x'=vee(ln(T'))
     */
    void update(const Eigen::Ref<const MatX1> &dx);
    virtual const Eigen::Ref<const MatX1> get_state() const {return x_;};
    // function returning the transformation
    virtual const Eigen::Ref<const MatX> get_stateT() const {return Tx_.T();};
    virtual const Eigen::Ref<const MatX1> get_last_linearization_state() const {return linearization_x_;};
    virtual const Eigen::Ref<const MatX1> get_last_deltaX() const {return dx_;};
    void print() const;

  protected:
    Mat61 x_;
    SE3 Tx_;//redundant state representation, now directly in SE(3)
    Mat61 linearization_x_;
    Mat61 dx_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen


};


}


#endif /* NODEPOSE3D_HPP_ */
