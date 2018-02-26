/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * node.hpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef NODE_HPP_
#define NODE_HPP_

#include "factor.hpp"
#include "Eigen/Dense"

namespace skmr{

class Node{
  public:
    Node();
    virtual ~Node() = 0;
    virtual void update(const Eigen::MatrixXd &dx) = 0;
  protected:
    Eigen::MatrixXd x_;
    std::vector<skmr::Factor*> factors_;
};

}



#endif /* NODE_HPP_ */
