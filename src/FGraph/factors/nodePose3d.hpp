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

#include "node.hpp"
#include "matrixBase.hpp"
#include "SE3.hpp" //requires including and linking SE3 library

namespace fg{

class NodePose3d : public Node
{
  public:
    /**
     * For initialization, requires an initial estimation of the state.
     */
    NodePose3d(const Mat61 &initial_x);
    virtual ~NodePose3d();
    /**
     * The update operation corresponds to the augmented sum, which is equivalent
     * to T'=exp(dxi^)*T and x'=vee(ln(T'))
     */
    void update(const Eigen::Ref<const MatX1> &dx);
    virtual const Eigen::Ref<const MatX1> getState() const {return x_;};
    void print() const;

  protected:
    Mat61 x_;
    lie::SE3 Tx_;//redundant state representation, now directly in SE(3)

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen


};


}


#endif /* NODEPOSE3D_HPP_ */
