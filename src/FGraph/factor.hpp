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

#include <vector>
#include "node.hpp"
#include "matrixBase.hpp"

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
    Factor(uint_t dim, uint_t allNodesDim, uint_t potNumberNodes = 5);
    virtual ~Factor();
    /**
     * Evaluates residuals and Jacobians
     */
    virtual void evaluate() = 0;
    /**
     * Jacobians are not evaluated, just the residuals
     */
    virtual matData_t evaluateError() = 0;

    /**
     * The print utility could be reimplemented on child classes
     * if there are special needs
     */
    virtual void print() const {};
    /**
     * Return a Ref to a dynamic matrix, while the child matrix should declare
     * all these variables as fixed size matrices, and ref takes care of
     * doing the conversion with minimal temporary artifacts
     */
    virtual const Eigen::Ref<const MatX1> getObs() const = 0;
    virtual const Eigen::Ref<const MatX1> getResidual() const = 0;
    virtual const Eigen::Ref<const MatX> getInvCovariance() const = 0;
    virtual const Eigen::Ref<const MatX> getWT2() const = 0;
    // TODO test this with MatX pointers, might be faster and no Ref is needed
    virtual const Eigen::Ref<const MatX> getJacobian() const = 0;

    //matData_t getChi2() const { return r_.dot(W_*r_);};//TODO do we need to calculate this?
    matData_t getChi2() const { return chi2_;};

    id_t getId() const {return id_;};
    void setId(id_t id) {id_ = id;};
    uint_t getDim() const {return dim_;};
    uint_t getAllNodesDim(){ return allNodesDim_;};
    const std::vector<std::shared_ptr<Node> >*
            getNeighbourNodes(void) const {return &neighbourNodes_;};

protected:
    id_t id_;
    /**
     * This is a sorted list, so at the constructor we should check
     * of the order based on increasing ids (See examples)
     */
    std::vector<std::shared_ptr<Node> > neighbourNodes_;
    uint_t dim_;//dimension of the observation
    uint_t allNodesDim_;//summation of all the nodes that the factor affects
    matData_t chi2_;

    // variables to declare on child Factor, for instance of dim 6
    //Mat61 obs_, r_; //and residuals
    //Mat6 J1_,J2_;//Jacobians
    //Mat6 W_;//inverse of observation covariance (information matrix)
    //Mat6 WT2_;//transpose and squared root of W.

};

}

#endif /* FACTOR_HPP_ */
