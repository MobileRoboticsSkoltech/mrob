/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * factor.hpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR_HPP_
#define FACTOR_HPP_

#include "Eigen/Dense"
#include <vector>
#include "node.hpp"

namespace fg{

/**
 * Factor class is a base pure abstract class defining factors,
 * the second type of vertexes on factor graphs (bipartite).
 * Factors keep track of all their neighbour nodes they are connected to.
 *
 * Because the number of Nodes they point to is fixed, we only allow
 * to indicate its node neighbours at the object declaration.
 * On the abstract class constructor they are not indicated, but should
 * be on any child class.
 */

class Factor{
public:
    /**
     * On the derived class constructor we will specify the (ordered)
     * nodes that the factor is connected to.
     */
    Factor(uint_t potNumberNodes = 5);
    virtual ~Factor();
    /**
     * Evaluates residuals and Jacobians
     */
    virtual void evaluate() = 0;
    /**
     * Jacobians are not evaluated, just the residuals
     */
    virtual matData_t evaluateError() = 0;

    virtual void print() const = 0;
    /**
     * Optimization on the compiler copying elision, so it wont be copied
     * XXX Is this true when we assign to a fixed matrix too?
     */
    MatX1 getObs() const {return obs_;};
    MatX getJacobian() const {return J_;};
    MatX getCovariance() const {return W_;};

    int getDim() const {return dim_;};
    const std::vector<std::shared_ptr<Node> >*
            getNeighbourNodes(void) const {return &neighbourNodes_;};
protected:
    std::vector<std::shared_ptr<Node> > neighbourNodes_;
    uint_t dim_;
    uint_t nodes_dim_;//summation of all the nodes that the factor affects

    matData_t chi2_;
    // declared here but initialized on child classes
    MatX1 obs_, r_; //and residuals
    MatX J_;//Joint Jacobian
    MatX W_;//inverse of observation covariance (information matrix)
};

}

#endif /* FACTOR_HPP_ */
