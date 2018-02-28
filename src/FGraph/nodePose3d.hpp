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

namespace skmr{

class NodePose3d : public Node
{
  public:
    typedef Eigen::Matrix<matData_t, 6, 1> StateVect;
    /**
     * For initialization, requires an initial estimation of the state.
     */
    NodePose3d(int id, StateVect &initial_x);
    virtual ~NodePose3d();
    virtual int getDim() const {return dim_;};
    void update(StateVect &dx);

  protected:
    int dim_;
    StateVect x_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}


#endif /* NODEPOSE3D_HPP_ */
