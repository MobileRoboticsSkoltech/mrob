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

#include <vector>
#include <memory>
#include "Eigen/Dense"
#include "matrixBase.hpp"

namespace skmr{

class Factor;

/**
 * Node class is an abstract class for creating future nodes. Pure
 * abstract methods on get and set dimension
 * is just a reminder that this is an Abstract class
 *
 * Destructor takes care of the neighbour factors included.
 *
 */

class Node{
  public:
    Node(unsigned int id, unsigned int potNumberFactors = 5);
    virtual ~Node();
    virtual int getDim(void) const = 0; //This will depend on the derived node
    unsigned int getId(void) const {return id_;};
    /**
     * Adds a factor to the connected factors in this node)
     */
    void addFactor(std::shared_ptr<Factor> &factor);
    void rmFactor(std::shared_ptr<Factor> &factor);
    const std::vector<std::shared_ptr<Factor> >*
            getNeighbourFactors(void) const {return &neighbourFactors_;};
  protected:
    unsigned int id_;
    std::vector<std::shared_ptr<Factor> > neighbourFactors_;
};

}



#endif /* NODE_HPP_ */
