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


namespace fg{

class Factor;

/**
 * Node class is an abstract class for creating future nodes. Pure
 * abstract methods on get and set dimension
 * is just a reminder that this is an Abstract class
 *
 * Node class keeps track of all the neighbouring factors.
 * Destructor takes care of the neighbour factors included.
 *
 */

class Node{
  public:
    Node(uint_t potNumberFactors = 5);
    virtual ~Node();
    virtual int getDim(void) const = 0; //This will depend on the derived node
    /**
     * Adds a factor to the list of factors connected to this node.
     */
    virtual bool addFactor(std::shared_ptr<Factor> &factor);
    /**
     * This function is very inefficient: it is an exhaustive search
     * so use only when necessary.
     */
    virtual bool rmFactor(std::shared_ptr<Factor> &factor);
    const std::vector<std::shared_ptr<Factor> >*
            getNeighbourFactors(void) const {return &neighbourFactors_;};
  protected:
    std::vector<std::shared_ptr<Factor> > neighbourFactors_;
};

}



#endif /* NODE_HPP_ */
