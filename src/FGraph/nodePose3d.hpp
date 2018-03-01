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
#include <Eigen/Dense>
#include "matrixBase.hpp"
#include "SE3.hpp" //requires including and linking SE3 library

namespace fg{

class NodePose3d : public Node
{
  public:
    /**
     * For initialization, requires an initial estimation of the state.
     */
    NodePose3d(unsign_t id, const Mat61 &initial_x);
    virtual ~NodePose3d();
    virtual int getDim() const {return dim_;};
    /**
     * The update operation corresponds to the augmented sum, which is equivalent
     * to T'=exp(dxi^)*T and x'=vee(ln(T'))
     */
    void update(const Mat61 &dx);
    Mat61 getState() const {return x_;};

  protected:
    int dim_;
    Mat61 x_;
    lie::SE3 Tx_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}


#endif /* NODEPOSE3D_HPP_ */
