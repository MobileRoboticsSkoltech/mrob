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


protected:
    MatX1 J_;//Jacobian
    MatX H_; // Hessian matrix, dense since it connects all poses from where plane was observed
    std::map<uint_t, Mat4> S_;  // According to our notation S = sum p*p'

    Mat41 planeEstimation_;
};

}
#endif /* PLANE_FACTOR_HPP_ */
