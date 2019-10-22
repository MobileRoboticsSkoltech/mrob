/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * plane_factor.hpp
 *
 *  Created on: Aug 16, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef PLANE_FACTOR_HPP_
#define PLANE_FACTOR_HPP_


#include "mrob/factor.hpp"
#include <map> // used for storing matrices S, paired by node Ids (or might be pointers)


namespace mrob{

/**
 * Plane factor is a vertex that complies with the Fgraph standards
 * and inherits from base factor.hpp
 *
 * The Plane factor connects different poses that have observed the same geometric entity.
 * It is not required an explicit parametrization of the plane, so the resultant topology
 * is N nodes connecting to the plane factor.
 *
 * In order to build the problem we would follow the interface specifications by FGraph
 * but we need extra methods and variables to keep track of the neighbours
 *
 * This class assumes that matrices S = sum p*p' are calculated before since they are directly inputs
 * XXX should we store all points?
 */
class PlaneFactor : public Factor{
public:
    /**
     * Creates a plane. The minimum requirements are 1 pose.
     */
    PlaneFactor(const Mat4 &S, std::shared_ptr<Node> &nodeOrigin);
    ~PlaneFactor();
    /**
     * Jacobians are not evaluated, just the residuals
     */
    void evaluate_residuals() override;
    /**
     * Evaluates residuals and Jacobians
     */
    void evaluate_jacobians() override;
    void evaluate_chi2() override;

    void print() const;

    // TODO are this functions useful? maybe add some assert inside
    const Eigen::Ref<const MatX1> get_obs() const
            {assert(0 && "PlaneFactor: method should not be called");return Mat31::Zero();};
    const Eigen::Ref<const MatX1> get_residual() const
            {assert(0 && "PlaneFactor: method should not be called");return Mat31::Zero();};
    const Eigen::Ref<const MatX> get_information_matrix() const
            {assert(0 && "PlaneFactor: method should not be called");return Mat4::Zero();};
    const Eigen::Ref<const MatX> get_trans_sqrt_information_matrix() const
            {assert(0 && "PlaneFactor: method should not be called");return Mat4::Zero();};
    const Eigen::Ref<const MatX> get_jacobian() const {return J_;};


    // NEW functions added to the base class factor.hpp
    /**
     * get plane returns the current planeEstimation
     */
    Mat41 get_plane(void) {return planeEstimation_;};
    /**
     * Add observation adds the S matrix and the time
     */
    void add_observation(const Mat4& S, std::shared_ptr<Node> &newNode);
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


protected:
    MatX1 J_;//Jacobian
    MatX H_; // Hessian matrix, dense since it connects all poses from where plane was observed
    std::map<uint_t, Mat4> S_;  // According to our notation S = sum p*p'

    Mat41 planeEstimation_;
    double planeError_;
    std::vector<Mat4> matrixQ_;
    Mat4 accumulatedQ_;//Q matrix of accumulated values for the incremental update of the error.
};

}
#endif /* PLANE_FACTOR_HPP_ */
