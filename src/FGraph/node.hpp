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

namespace fgraph{

class Node{
  public:
    Node();
    virtual ~Node() = 0;
    virtual void update() = 0;
    virtual void evaluate() = 0;
    virtual void evaluateJacobians() = 0;
  protected:
    double X;
};

}



#endif /* NODE_HPP_ */
