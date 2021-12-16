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
#include <unordered_map>
#include <deque>
#include <Eigen/StdVector>


namespace mrob{

/**
 * Eigen factor Plane is a vertex that complies with the Fgraph standards
 * and inherits from base EigenFactor.hpp
 *
 * The Plane factor connects different poses that have observed the same geometric entity.
 * It is not required an explicit parametrization of the plane, so the resultant topology
 * is N nodes connecting to the plane factor.
 *
 * NOTE: due to its nature, multiple observation can be added to the same EF,
 * meaning we need to create a constructor PLUS an additional method
 *  - add_point()
 *
 * In order to build the problem we would follow the interface specifications by FGraph
 * but we need extra methods and variables to keep track of the neighbours
 *
 * This class assumes that matrices S = sum p*p' are calculated before since they are directly inputs
 * XXX should we store all points?
 */
class EigenFactorPlane: public EigenFactor{
public:
    /**
     * Creates an Eigen Factor plane. The minimum requirements are 1 pose, which is not required
     * at this stage, but will be introduced when we add points/Q matrix.
     */
    EigenFactorPlane(Factor::robustFactorType robust_type = Factor::robustFactorType::QUADRATIC);
    ~EigenFactorPlane() override = default;
    /**
     * Jacobians are not evaluated, just the residuals.
     * This function is calculating the current plane estimation
     */
    void evaluate_residuals() override;
    /**
     * Evaluates Jacobians, given the residual evaluated
     * It also evaluated the Hessians
     */
    void evaluate_jacobians() override;
    /**
     * Chi2 is a scaling of the plane error
     */
    void evaluate_chi2() override;

    void print() const;

    MatRefConst get_obs() const
            {assert(0 && "EigenFactorPlane:get_obs: method should not be called");return Mat31::Zero();}
    VectRefConst get_residual() const
            {assert(0 && "EigenFactorPlane::get_resigual: method should not be called");return Mat31::Zero();}
    MatRefConst get_information_matrix() const
            {assert(0 && "EigenFactorPlane::get_inform method should not be called");return Mat4::Zero();}

    /**
     * get jacobian returns the jacobian corresponding to the given node id.
     * @return
     */
    MatRefConst get_jacobian([[maybe_unused]] mrob::factor_id_t id = 0) const override;
    /**
     * get hessian returns the Hassian corresponding to the given node id.
     * @return
     */
    MatRefConst get_hessian(mrob::factor_id_t id = 0) const override;


    // NEW functions added to the base class factor.hpp
    /**
     * get current state of the Eigen Factor, in this case,
     * a plane is returned as the current planeEstimation
     */
    VectRefConst get_state(void) const override
                {return planeEstimation_;}
    /**
     * Add point: raw points are stored into an unoderred map
     *
     * Later, the S matrix should be calculated
     *
     * The alternative is adding directly S, but this offers less
     * flexibility.
     * XXX adding one by one might be inefficient
     */
    void add_point(const Mat31& p, std::shared_ptr<Node> &node, matData_t &W) override;
    /**
     * get mean point calculates the mean of the pointcloud observed at pose node id,
     * given that S = sum p * p' =  sum ([x2 xy xz x
     *                                    yx y2 yz y
     *                                    zx zy z2 z
     *                                    x   y  z 1]
     * ser we just calculate S and return
     */
    Mat31 get_mean_point(factor_id_t id);


protected:
    /**
     * Estimates the plane parameters: v = [n', d]'_{4x1}, where v is unit, (due to the Eigen solution)
     * although for standard plane estimation we could enforce unit on the normal vector n.
     */
    double estimate_plane();
    /**
     * Calculates the matrix S = sum(p*p'), where p = [x,y,z,1]
     * for all planes, as an aggregation of the outer product of all
     * homogeneous points
     * If reset = true, clears all information and recalculates S
     * If reset = false (default) only calculates S if there is no calculation yet
     */
    void calculate_all_matrices_S(bool reset=false);
    /**
     *  calculates the matrix Qi = 1^T_i * Si * (1^T_i)^transp
     *  for all planes. Since this is an iterative process on T's,
     *  we separate the calculation of the S matrix,
     *  and the Q matrix which rotates S
     */
    void calculate_all_matrices_Q();
    /**
     * A deque storing the ids in the FGraph of each of the poses
     * This is preferred over vector due to the a priori unknown size of
     * elements. The same applies for other structures containing matrices
     */
    std::deque<factor_id_t> nodeIds_;
    /**
     * Mapping from the FG nodes IDs to the local indexes in the current EF.
     * This conversion is necessary to maintain consistency in case of
     * non-subsequent observations
     */
    std::unordered_map<factor_id_t, uint_t> reverseNodeIds_;
    /**
     * The Jacobian of the plane error, the poses involved.
     * Stores the map according to the nodes indexes/identifiers.
     */
    std::deque<Mat61, Eigen::aligned_allocator<Mat61>> J_;
    /**
     * Hessian matrix, dense since it connects all poses from where plane was observed.
     * We store the block diagonal terms, according to the indexes of the nodes
     */
    std::deque<Mat6, Eigen::aligned_allocator<Mat6>> H_;
    /**
     * According to our notation S = sum p*p'
     * We choose unordered map here since this is a subset of neighbours (small) and we will iterate over them
     * Iterations may be not in strict order, but we don't care much for now (will we?)
     *
     * Q = T *S *T'
     */
    std::deque<Mat4, Eigen::aligned_allocator<Mat4>> S_, Q_;
    Mat4 accumulatedQ_;//Q matrix of accumulated values for the incremental update of the error.

    Mat41 planeEstimation_;
    matData_t planeError_; //this is chi2 scaled by the covariance of point measurement.

    // subset of pointcloud for the given plane
    //std::unordered_map<factor_id_t, std::vector<Mat31> > allPlanePoints_;
    std::deque<std::deque<Mat31, Eigen::aligned_allocator<Mat31>> > allPlanePoints_;
    std::deque<std::deque<matData_t> > allPointsInformation_;
    uint_t numberPoints_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen
};

}
#endif /* EigenFactorPlane_HPP_ */
