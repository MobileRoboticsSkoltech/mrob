/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
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

#include "mrob/matrix_base.hpp"
#include "mrob/factor.hpp"

namespace mrob{

class Factor;

/**
 * Node class is an abstract class for creating future nodes. Pure
 * abstract methods on get and set dimension
 * is just a reminder that this is an Abstract class
 *
 * Node class keeps track of all the neighbouring factors.
 * Destructor takes care of the neighbour factors included.
 *
 * States are indicated as references to MatX matrices, since
 * it can be either a block vector or a matrix (transformation matrix)
 *
 * For updates it will always be a block vector : Ref < MatX1 >
 *
 *	Two states are kept at the same time:
 *	- (principal) state: used for factors evaluations, errors and Jacobians
 *	- auxiliary state: a book-keep state useful for partial updates
 */

class Node{
  public:
    Node(uint_t dim, uint_t potNumberFactors = 5);
    virtual ~Node();
    /**
     * The update function, given any block vector it updates
     * the value of the state (principal).
     *
     * Since we don't know the size at compilation, we declare
     * a dynamic matrix, but on run-time we would like to use
     * a fixed block matrix, and this virtual function will handle
     * it nicely.
     */
    virtual void update(const Eigen::Ref<const MatX1> &dx) = 0;
    /**
     * Updates *FROM* the auxiliary state the principal state.
     */
    virtual void update_from_auxiliary(const Eigen::Ref<const MatX1> &dx) = 0;
    /**
     * At run time sets the new value of the estate and auxiliary to be x
     *
     */
    virtual void set_state(const Eigen::Ref<const MatX> &x) = 0;
    /**
     * New auxiliary state set
     */
    virtual void set_auxiliary_state(const Eigen::Ref<const MatX> &x) = 0;
    /**
     * Declared as a dynamic matrix reference to allow any size to be returned.
     * At run time returns a Reference to a fixed size matrix and provide
     * it as an argument for the getState function, no need to be dynamic,
     * as long as the dimension is correctly set
     */
    virtual const Eigen::Ref<const MatX> get_state() const = 0;
    /**
     * returns the state Transformation, equivalent to state
     * but direcly the matrix representing the rotation or RBT
     TODO to remove, states could be considered matrices with new api
     */
    //virtual const Eigen::Ref<const MatX> get_stateT() const = 0;
    /**
     * Returns a matrix to the last auxiliary state. This data structure is for the incre-
     * metal implementation, or for error evaluation
     */
    virtual const Eigen::Ref<const MatX> get_auxiliary_state() const = 0;
    virtual void print() const {};
    uint_t get_id() const {return id_;};
    void set_id(uint_t id) {id_ = id;};
    uint_t get_dim(void) const {return dim_;};
    /**
     * Adds a factor to the list of factors connected to this node.
     */
    virtual bool add_factor(std::shared_ptr<Factor> &factor);
    /**
     * This function is very inefficient: it is an exhaustive search
     * so use only when necessary.
     */
    virtual bool rm_factor(std::shared_ptr<Factor> &factor);
    void clear() {neighbourFactors_.clear();};
    const std::vector<std::shared_ptr<Factor> >*
            get_neighbour_factors(void) const {return &neighbourFactors_;};

  protected:
    // no oder needed here
    std::vector<std::shared_ptr<Factor> > neighbourFactors_;
    uint_t id_;
    uint_t dim_;
    /**
     * On this pure abstract class we can't define a vector state,
     * but we will return and process Ref<> to dynamic matrices.
     * The reason for doing that is polymorphism. We prefer to
     * preserve polymorphism for node storage at the cost of
     * a returning Ref, although there is no extra allocation on this process.
     * For instance, we will declare:
     *      Mat61 x_;
     * or   SE3 T_; (for transformations)
     */
};

/**
 * utility function for 2D poses to wrap angles into [-pi,pi]
 */
double wrap_angle(double angle);


}



#endif /* NODE_HPP_ */
