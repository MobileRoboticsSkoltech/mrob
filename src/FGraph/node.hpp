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
     * We may declare a fixed size matrix at run time and provide
     * it as an argument for the getState function, no need to be dynamic,
     * as long as the dimension is correctly set
     */
    void getState(Eigen::Ref<MatX1> res) const {res = x_;};
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
    void print() const ;
  protected:
    // For highly connected nodes where removing is necessary, map should be better
    std::vector<std::shared_ptr<Factor> > neighbourFactors_;
    id_t id_;
    /**
     * We have chosen to use a dynamic matrix, instead of
     * templating the dimensions and creating a fixed vector.
     * The reason for doing that is polymorphism, we prefer to
     * preserve polymorphism for node storage at the cost of
     * a slightly less allocation of resources.
     */
    MatX1 x_;
    uint_t dim_;
};

}



#endif /* NODE_HPP_ */
