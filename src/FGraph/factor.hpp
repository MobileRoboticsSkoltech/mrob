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

    /**
     * The print utility could be reimplemented on child classes
     * if there are special needs
     */
    virtual void print() const;
    /**
     * Return a constant pointer to the dynamic matrix. At runtime we still
     * can't know the size in all problems. TODO If we were using using a fixed sized matrix
     * we probably want to use an eigen reference (Ref) to a dynamic matrix, e.g.
     *     Mat5 J = getJacobian();
     */
    const MatX1* getObs() const {return &obs_;};
    const MatX1* getResidual() const {return &r_;};
    const MatX* getCovariance() const {return &W_;};
    const MatX* getJacobian() const {return &J_;};
    //void getJacobian(Eigen::Ref<MatX> res) const {res = J_;};

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
    std::vector<std::shared_ptr<Node> > neighbourNodes_;
    uint_t dim_;
    uint_t allNodesDim_;//summation of all the nodes that the factor affects
    matData_t chi2_;

    // declared here but initialized on child classes
    MatX1 obs_, r_; //and residuals
    MatX J_;//Joint Jacobian
    MatX W_;//inverse of observation covariance (information matrix)
};

}

#endif /* FACTOR_HPP_ */
