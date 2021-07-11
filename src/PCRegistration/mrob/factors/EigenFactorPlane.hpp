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
 * EigenFactorPlane.hpp
 *
 *  Created on: Aug 16, 2019
 *  Created on: July 10, 2021 (for real)
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef EIGENFACTORPLANE_HPP_
#define EIGENFACTORPLANE_HPP_


#include "mrob/factor.hpp"
#include <unordered_map> // used for storing matrices S, paired by node Ids (or might be pointers)


namespace mrob{

/**
 * Eigen factor Plane is a vertex that complies with the Fgraph standards
 * and inherits from base factor.hpp
 *
 * The Plane factor connects different poses that have observed the same geometric entity.
 * It is not required an explicit parametrization of the plane, so the resultant topology
 * is N nodes connecting to the plane factor.
 *
 * NOTE: due to its nature, multiple observation can be added to the same EF,
 * meaning we need to create a constructor PLUS an additional method
 *  - add_observation()
 *
 * In order to build the problem we would follow the interface specifications by FGraph
 * but we need extra methods and variables to keep track of the neighbours
 *
 * This class assumes that matrices S = sum p*p' are calculated before since they are directly inputs
 * XXX should we store all points?
 */
class EigenFactorPlane: public Factor{
public:
    /**
     * Creates a plane. The minimum requirements are 1 pose.
     */
    EigenFactorPlane(const Mat4 &S, std::shared_ptr<Node> &nodeOrigin,
            Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~EigenFactorPlane() override = default;
    /**
     * Jacobians are not evaluated, just the residuals
     */
    void evaluate_residuals() override {}
    /**
     * Evaluates residuals and Jacobians
     */
    void evaluate_jacobians() override {}
    void evaluate_chi2() override {}

    void print() const;

    const Eigen::Ref<const MatX> get_obs() const
            {assert(0 && "EigenFactorPlane:get_obs: method should not be called");return Mat31::Zero();}
    const Eigen::Ref<const MatX1> get_residual() const
            {assert(0 && "EigenFactorPlane::get_resigual: method should not be called");return Mat31::Zero();}
    const Eigen::Ref<const MatX> get_information_matrix() const
            {assert(0 && "EigenFactorPlane::get_inform method should not be called");return Mat4::Zero();}
    const Eigen::Ref<const MatX> get_jacobian() const
            {assert(0 && "EigenFactorPlane::get_jacobian: method should not be called");return Mat61::Zero();}


    // NEW functions added to the base class factor.hpp
    /**
     * get plane returns the current planeEstimation
     */
    Mat41 get_plane(void) {return planeEstimation_;};
    /**
     * Add observation adds the S matrix and the time
     */
    void add_observation(const Mat4& S, std::shared_ptr<Node> &newNode);
    //void add_observation(const Mat4& S, time ts, SE3 initialPose); // TODO variables for these non-optimized nodes
    /**
     * Estimates the plane parameters: v = [n', d]'_{4x1}, where v is unit, (due to the Eigen solution)
     * although for standard plane estimation we could enforce unit on the normal vector n.
     */
    double estimate_plane();
    /**
     * Estimates Incremetally the plane parameters: v = [n', d]'_{4x1}, where v is unitary, (due to the SVD solution)
     * although for standard plane estimation we could enforce unitary on the normal vector n.
     *
     * The difference with the previous estimate_plane() is that we update the matrix Q for the give time
     * stamp and recalculate the solution, on constant time O(1)
     */
    double estimate_plane_incrementally(uint_t t);
    /**
     * get error: returns the error as the min eigenvalue
     */
    double get_error() const {return planeError_;};
    /**
     * get error incremental: returns the error as the min eigenvalue only updating the
     * value of Q_t, at time step t. Nothing inside gets updated
     */
    double get_error_incremental(uint_t t) const;
    /**
     *  calculates the matrix Qi = 1^T_i * Si * (1^T_i)^transp
     *  for all planes. Since this is an iterative process on T's,
     *  we separate the calculation of the S matrix,
     *  and the Q matrix which rotates S
     */
    void calculate_all_matrices_Q();
    /**
     * get mean point calculates the mean of the pointcloud observed at time t,
     * given that S = sum p * p' =  sum ([x2 xy xz x
     *                                    yx y2 yz y
     *                                    zx zy z2 z
     *                                    x   y  z 1]
     * ser we just calcualte S and return
     */
    //Mat31 get_mean_point(uint_t t); // XXX is this necessary?
    /**
     * calculate Jacobian at node nodeId, returns the Jacobian over the Eigendecomposition of Q  = V D V'
     * as d lamda = v' * dQ * v
     */
    Mat61 calculate_jacobian(uint_t nodeId);
    /**
     * Calculate Hessian at time
     */


protected:
    /**
     * On the base class it is declared vector<std::shared_ptr<Node> > neighbourNodes_;
     * This is a sorted list, so the add_node function makes sure to keep the order.
     *
     * For planes nodes we need to keep a sorted list of nodes. While
     * other factors connect to 1 node, 2 usually, plane factor is not bounded,
     * so we will store a sorted container for nodes that have observed the same
     * plane. Nodes on this list will not be optimized, but they should be related to its 2 closes nodes.
     *
     * For optimizaing we will use the neighbourNodes_ from the base class, and here we must preserve
     * the order.
     */
    std::vector<std::shared_ptr<Node> > planeNodes_;
    /**
     * The Jacobian of the plane error, the poses involved.
     * Stores the map according to the nodes indexes/identifiers.
     */
    std::unordered_map<factor_id_t, Mat61> J_;
    /**
     * Hessian matrix, dense since it connects all poses from where plane was observed.
     * We store the block diagonal terms, according to the indexes of the nodes
     */
    std::unordered_map<factor_id_t, Mat6> H_;
    /**
     * According to our notation S = sum p*p'
     * We choose unordered map here since this is a subset of neighbours (small) and we will iterate over them
     * Iterations may be not in strict order, but we don't care much for now (will we?)
     *
     * Q = T *S *T'
     */
    std::unordered_map<factor_id_t, Mat4> S_, Q_;
    Mat4 accumulatedQ_;//Q matrix of accumulated values for the incremental update of the error.

    Mat41 planeEstimation_;
    matData_t planeError_; //this is chi2 scaled by the covariance of point measurement.

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen
};

}
#endif /* EigenFactorPlane_HPP_ */
