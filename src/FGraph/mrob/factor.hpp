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

#include "mrob/matrix_base.hpp"
#include "mrob/node.hpp"

namespace mrob{

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
    virtual matData_t evaluate_error() = 0;

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
    virtual const Eigen::Ref<const MatX1> get_obs() const = 0;
    virtual const Eigen::Ref<const MatX1> get_residual() const = 0;
    virtual const Eigen::Ref<const MatX> get_information_matrix() const = 0;
    virtual const Eigen::Ref<const MatX> get_trans_sqrt_information_matrix() const = 0;
    /**
     * get_jacobian returns a block matrices stacking all the Jacobians on the factor.
     * The convention is that Jacobians corresponding to
     *
     * TODO test this. Do we really need an ordered Jacobian??
     */
    virtual const Eigen::Ref<const MatX> get_jacobian() const = 0;

    //matData_t getChi2() const { return r_.dot(W_*r_);};//TODO do we need to calculate this?
    matData_t get_chi2() const { return chi2_;};

    id_t get_id() const {return id_;};
    void set_id(id_t id) {id_ = id;};
    uint_t get_dim() const {return dim_;};
    uint_t get_all_nodes_dim(){ return allNodesDim_;};
    const std::vector<std::shared_ptr<Node> >*
            get_neighbour_nodes(void) const {return &neighbourNodes_;};

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
