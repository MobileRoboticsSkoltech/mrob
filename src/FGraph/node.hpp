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
#include <assert.h>
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
    Node(uint_t dim, uint_t potNumberFactors = 5);
    virtual ~Node();
    /**
     * The update operation, give a block vector, it updates
     * the value of the state x.
     * Since we don't know the size at compilation, we declare
     * a dynamic matrix, but on run-time we would like to use
     * a fixed block matrix, and this virtual function will handle
     * it nicely.
     */
    virtual void update(const Eigen::Ref<const MatX1> &dx) = 0;
    /**
     * We will return a Reference to a fixed size matrix at run time and provide
     * it as an argument for the getState function, no need to be dynamic,
     * as long as the dimension is correctly set
     */
    virtual const Eigen::Ref<const MatX1> getState() const = 0;
    virtual void print() const {};
    id_t getId() const {return id_;};
    void setId(id_t id) {id_ = id;};
    uint_t getDim(void) const {return dim_;};
    /**
     * Adds a factor to the list of factors connected to this node.
     */
    virtual bool addFactor(std::shared_ptr<Factor> &factor);
    /**
     * This function is very inefficient: it is an exhaustive search
     * so use only when necessary.
     */
    virtual bool rmFactor(std::shared_ptr<Factor> &factor);
    void clear() {neighbourFactors_.clear();};
    std::vector<std::shared_ptr<Factor> >*
            getNeighbourFactors(void) {return &neighbourFactors_;};

  protected:
    // For highly connected nodes where removing is necessary, map should be better
    std::vector<std::shared_ptr<Factor> > neighbourFactors_;
    id_t id_;
    uint_t dim_;
    /**
     * On this pure abstract class we can't define a vector state,
     * but we will return and process Ref<> to dynamic matrices.
     * The reason for doing that is polymorphism. We prefer to
     * preserve polymorphism for node storage at the cost of
     * a returning Ref, although there is no extra allocation on this process.
     * For instance, we will declare:
     *      Mat61 x_;
     */
};

}



#endif /* NODE_HPP_ */
