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
#include <memory>

#include "mrob/matrix_base.hpp"
#include "mrob/node.hpp"

namespace mrob{

class Node;

/**
 * Factor class is a base pure abstract class defining factors,
 * the second type of vertexes on factor graphs (bipartite).
 * Factors keep track of all their neighbour nodes they are connected to.
 *
 * By convention, the residuals r_i are ALWAYS formulated as follows:
 *
 * -------------------------------------------
 * |           r(x) =  h(x) - z              |
 * -------------------------------------------
 *
 * otherwise the optimization will not work properly.
 *
 * Because the number of Nodes they point to is fixed, we only allow
 * to indicate its node neighbours at the object declaration.
 * On the abstract class constructor they are not indicated, but should
 * be on any child class.
 *
 * Constructor functions will be overloaded to include the pointers of the nodes,
 * The convention is from node 1, to node 2, and etc. such that: myfactor2poses(n1, n2, ...)
 *
 * Conventions (transparent for users, but good to know):
 * - The connecting nodes are stores ordered on a vector. The user does not *have*
 *   to provide such ordering, but at creation each factor such comply this.
 *
 * - Jacobian is a block matrix corresponding to the Jacobian of the first node J1,
 *   then second node J2, etc, such that J = [J1, J2, ..., Jn],
 *
 * Robust Factors TODO
 *  All factors also allow for robust factors, they just have to be specified at the constructor.
 *  By the default the robust factor is quadratic (no effect)
 *  Currently supports robust factors:
 *                   Influence function                 robust weight
 *                   --------------------             --------------------
 *   - Quadratic:          p(u) = 1/2u^2              w(u)  = 1
 *   - Huber:     p(u) = 1/2u^2    if u < d            w(u) = 1
                          d(u-1/2d)                          1/u
 *   - Cauchy:   p(u) = 1/2 ln(1+u^2)                  w(u) = 1/(1+u^2)
 *   - McClure:   p(u) = 1/2 u^2 / (1+u^2)              w(u) = 1/(1+u^2)^2
 *   - Ransac:   p(u) = 1/2u^2    if u < d            w(u) = 1
 *                      d                                    0
 */

class Factor{
public:
    /**
     * On the derived class constructor we will specify the (ordered)
     * nodes that the factor is connected to.
     */
    enum robustFactorType{QUADRATIC = 0, HUBER, CAUCHY, MCCLURE, RANSAC};
    Factor(uint_t dim, uint_t allNodesDim, robustFactorType factor_type = QUADRATIC, uint_t potNumberNodes = 5);
    virtual ~Factor();
    /**
     * Residuals are evaluated with respect to the current solution
     */
    virtual void evaluate_residuals() = 0;
    /**
     * Evaluates Jacobians, this also creates a new linearization point.
     * This function MOST likely needs to evaluate residuals first
     */
    virtual void evaluate_jacobians() = 0;
    /**
     * Evaluates chi2 of the current problem, with the given residuals.
     * It may be required to evaluate_residuals() to obtain the new chi2 values
     * This function MOST likely needs to evaluate residuals first, but
     * evaluate_residuals does not necessarily requires to calculate chi2, that is why
     * there are 2 functions.
     */
    virtual void evaluate_chi2() = 0;
    /**
     * get chi2 returns the value in the variable chi2_. This value will be updated
     * every time there is a calculation of residuals.
     */
    matData_t get_chi2() const { return chi2_;}
    /**
     * The print utility could be re-implemented on child classes
     * if there are special needs
     */
    virtual void print() const {}
    /**
     * Return a Ref to a dynamic matrix, while the child matrix should declare
     * all these variables as fixed size matrices, and ref takes care of
     * doing the conversion with minimal temporary artifacts
     * Observation can be a 3d point, a 3d pose (transformation 4x4), etc.
     */
    virtual const Eigen::Ref<const MatX> get_obs() const = 0;
    /**
     * Residual will always be a block vector
     */
    virtual const Eigen::Ref<const MatX1> get_residual() const = 0;
    virtual const Eigen::Ref<const MatX> get_information_matrix() const = 0;
    /**
     * get_jacobian returns a block matrices stacking all the Jacobians on the factor.
     * The convention is that Jacobians corresponding to
     *
     */
    virtual const Eigen::Ref<const MatX> get_jacobian() const = 0;


    factor_id_t get_id() const {return id_;}
    void set_id(factor_id_t id) {id_ = id;}
    uint_t get_dim() const {return dim_;}
    uint_t get_all_nodes_dim(){ return allNodesDim_;}
    const std::vector<std::shared_ptr<Node> >*
            get_neighbour_nodes(void) const {return &neighbourNodes_;}

    /**
     * Robust functions, given the current distance u = sqrt(r' W r)
     * It can be calculated in the base class for most of the robust functions
     * given that we provide the following inputs:
     *  - u = sqrt(r'Wr)
     *  - param: may be used to pass some information for some losses
     *
     * Output:
     *  - dp/du 1/u or influence by the inverse of the distance u.
     *  The output is directly multiplied in the formation of the
     *  matrix W when building the problem.
     */

    matData_t evaluate_robust_weight(matData_t u, matData_t params = 0.0);

protected:
    factor_id_t id_;
    /**
     * This is a sorted list, so at the constructor we should check
     * of the order based on increasing ids (See examples)
     */
    std::vector<std::shared_ptr<Node> > neighbourNodes_;
    uint_t dim_;//dimension of the observation
    uint_t allNodesDim_;//summation of all the nodes that the factor affects
    matData_t chi2_;

    // Robust factor weighting the "iteratively weighted LSQ"
    robustFactorType robust_type_;
    matData_t robust_weight_; // dp/du 1/u or influence by the inverse of the distance u

    /**
     * TODO: for ransac factors, we can think of the problem as an hypothesis rejection test
     * that the observation we have obtained actually belongs to a different distribution and hence
     * we remove it from our estimation.
     * we need a threshold (a ransac_significance level, e.g. 5%) and methods for setting it.
     * TODO values could be the two-sided test if we assume Gaussian distributions, one-sided, etc.
     * Note we need to account for the dimensionality of the node in oder to achive the p-value set.
     */
    //matData_t ransac_significance_level_;

    // variables to declare on child Factor, for instance of dim 6
    //Mat61 obs_, r_; //and residuals
    //Mat<Z,N> J_;//Jacobians
    //Mat6 W_;//inverse of observation covariance (information matrix)

};


}

#endif /* FACTOR_HPP_ */
