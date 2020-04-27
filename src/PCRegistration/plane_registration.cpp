/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * planeRegistration.cpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include "mrob/pc_registration.hpp"
#include "mrob/plane_registration.hpp"
#include <Eigen/LU> // for inverse and determinant
#include <iostream>


#include <chrono>

using namespace mrob;


PlaneRegistration::PlaneRegistration():
        numberPlanes_(0), numberPoses_(0),isSolved_(0), trajectory_(new std::vector<SE3>(8,SE3())),
        solveMode_(SolveMode::GRADIENT_DESCENT_NAIVE),
        c1_(1e-4), c2_(0.9), alpha_(0.75), beta_(0.1)
{
}
// XX is this constructor really necessary?
/*PlaneRegistration::PlaneRegistration(uint_t numberPlanes, uint_t numberPoses):
        numberPlanes_(numberPlanes), numberPoses_(numberPoses),isSolved_(0),
        trajMode_(TrajectoryMode::SEQUENCE), trajectory_(new std::vector<SE3>(numberPoses, SE3())),
        solveMode_(SolveMode::GRADIENT_DESCENT_NAIVE),
        c1_(1e-4), c2_(0.9), alpha_(0.75), beta_(0.1)
{
    planes_.reserve(numberPlanes);
    inverseHessian_.resize(numberPoses, 1e-3 * Mat6::Identity());
    previousJacobian_.resize(numberPoses, Mat61::Zero());
    previousState_.resize(numberPoses, Mat61::Zero());
}*/

PlaneRegistration::~PlaneRegistration()
{

}


void PlaneRegistration::set_number_planes_and_poses(uint_t numberPlanes, uint_t numberPoses)
{
    planes_.clear();
    planes_.reserve(numberPlanes);
    trajectory_->clear();
    trajectory_->resize(numberPoses, SE3());
    numberPlanes_ = numberPlanes;
    numberPoses_ = numberPoses;

    inverseHessian_.clear();
    inverseHessian_.resize(numberPoses, 1e-3 * Mat6::Identity());
    previousJacobian_.clear();
    previousJacobian_.resize(numberPoses, Mat61::Zero());
    previousState_.clear();
    previousState_.resize(numberPoses, Mat61::Zero());
}

uint_t PlaneRegistration::solve(bool singleIteration)
{
    // Initializitaion
    //inverseHessian_.clear();
    //inverseHessian_.resize(numberPoses, Mat6::Identity());
    //Mat61 previousJacobian = Mat61::Zero();


    // for benchmarking. This should go away later
    typedef std::chrono::microseconds Ttim;
    auto t1 = std::chrono::steady_clock::now();
    auto t2 = std::chrono::steady_clock::now();
    auto dif = std::chrono::duration_cast<Ttim>(t2 - t1);


    // iterative process, on convergence basis | error_k - error_k-1| < tol
    uint_t solveIters = 0;
    double previousError = 1e20, diffError = 10;
    do
    {

        // 1) calculate plane estimation given the current trajectory
        double  initialError = 0.0;
        for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
        {
            initialError += it->second->estimate_plane();
        }
        diffError = previousError - initialError;
        previousError = initialError;

        // 2) calculate Gradient = Jacobian^T. We maintain the nomenclature Jacobian for coherence on the project,
        //    but actually this Jacobian should be transposed.
        Mat61 jacobian;
        double  numberPoints;
        // it should start at t = 1 because t = 0 is the fixed reference frame T0 = I
        // Convergence greatly improves if we include the first pose as well.
        // We update it and then transform all the sequence according to T0^-1 to maintain T0 = I
        for (uint_t t = 0 ; t < trajectory_->size(); ++t)
        {
            jacobian.setZero();
            numberPoints = 0.0;
            for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
            {
                jacobian += it->second->calculate_jacobian(t);
                numberPoints += it->second->get_number_points(t);
                //std::cout << "Plane " << it->first << ", error = " << it->second->estimate_plane() << ", Jacobian = " << it->second->calculate_jacobian(t).transpose() << std::endl;
            }

            // 3) update results Ti = exp(-dxi) * Ti (our convention, we expanded from the left)
            // 3.1) gradient decent with fixed step upgrade alpha = 0.75/N
            if (solveMode_ == SolveMode::GRADIENT_DESCENT_NAIVE)
            {
                // after some tuning, best values for alpha = 0.3
                double alpha = alpha_/numberPoints;
                Mat61 dxi = -alpha * jacobian;// dxi = alpha * p_k = alpha *(-Grad f)
                //std::cout << "\njacobian : = " << jacobian.transpose() << ", at time step " << t <<  std::endl;
                trajectory_->at(t).update_lhs(dxi);

            }
            // 3.1-A) Incremental update after each pose
            // Results: does not improve much for a hug increase on the overhead
            if (solveMode_ == SolveMode::GRADIENT_DESCENT_INCR)
            {
                double alpha = alpha_/numberPoints;
                Mat61 dxi = -alpha * jacobian;// dxi = alpha * p_k = alpha *(-Grad f)
                //std::cout << "jacobian : = " << jacobian.norm() << std::endl;
                trajectory_->at(t).update_lhs(dxi);
                for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
                    //XXX we also return current new error lambda, it could be used
                    it->second->estimate_plane_incrementally(t);// this updates the current solution v. XXX: SLOW
            }

            // 3.1-B) Steepest gradient decent with fixed step upgrade alpha = 1/N
            // results: incredibly slow, useless
            if (solveMode_ == SolveMode::STEEPEST)
            {
                double alpha = 1e-2;
                Mat61 dxi = - alpha/jacobian.norm() * jacobian; //dxi = alpha * p_k = alpha *(-Grad f)/norm(Grad)
                trajectory_->at(t).update_lhs(dxi);
            }

            // 3.2) Heavy Ball (Poliak'64): Gradient decent with averaging: x_k+1 = x_k - alpha Grad + beta (xk - x_k-1)
            // Results: does not really improve much
            if (solveMode_ == SolveMode::HEAVYBALL)
            {
                double alpha = 1.0/numberPoints;
                Mat61 currentState = trajectory_->at(t).ln_vee();
                std::cout << "diff in state : "<< (currentState - previousState_[t]).norm() <<  ", alpha gradient : = " << (alpha * jacobian).norm() << std::endl;
                Mat61 dxi = -alpha * jacobian + 0.1 * (currentState - previousState_[t]);
                previousState_[t] = currentState;
                trajectory_->at(t).update_lhs(dxi);
            }

            // 3.3) Momentum (Hinton84?):
            //      Gradient decent where Dx_k = beta * Dx_k - alpha Grad
            //                                    and x_k+1 = x_k + D x_k
            // Results: works great
            if (solveMode_ == SolveMode::MOMENTUM)
            {
                double alpha = alpha_/numberPoints; // raw alpha = 1
                //double beta = 0.1;
                Mat61 dxi = -alpha * jacobian + beta_ * previousState_[t];
                previousState_[t] = dxi;
                std::cout << "diff in state : "<< dxi.norm() <<  ", alpha gradient : = " << (alpha * jacobian).norm() << std::endl;
                trajectory_->at(t).update_lhs(dxi);
            }

            // 3.3-B) Momentum with a given sequence of params: Gradient decent where D x_k = beta * D x_k - alpha Grad
            //                                    and x_k+1 = x_k + D x_k
            // Results: not implemented
            if (solveMode_ == SolveMode::MOMENTUM_SEQ)
            {
                // select a sequence of parameters
                double alpha = 1.0/numberPoints;
                double beta = 0.1; // adaptive depending on the gradient, sequence: 0.1, 0.5,0.9,0.99
                Mat61 dxi = -alpha * jacobian + beta * previousState_[t];
                previousState_[t] = dxi;
                std::cout << "diff in state : "<< dxi.norm() <<  ", alpha gradient : = " << (alpha * jacobian).norm() << std::endl;
                trajectory_->at(t).update_lhs(dxi);
            }

            // 3.4) Nesterov's Accelerated Gradient (NAG, by Nesterov): it can be understood as a momentum algorithm as well (see Sutskever'2013)
            // Results: we did not implement this due to
            //
            // 3.4-B) Bengio's NAG: a modification to NAG as proposed in Bengio-2013. Fixed parameters
            //          1) momentum or velocity  v_k = beta_k-1 v _k-1 - alpha_k-1 Grad f (x_k-1)
            //          2) x_k+1 = x_k + beta_k+1 beta_k * v_k - (1 + beta_k+1)*alpha_k * Grad f(x_k)
            // Restuls, works great, the default optimizer
            if (solveMode_ == SolveMode::BENGIOS_NAG)
            {
                double alpha = alpha_/numberPoints; //raw alpha 1.0
                //beta_ = 0.05;
                // x update
                Mat61 dxi = beta_ * beta_ * previousState_[t] - (1 + beta_) * alpha * jacobian;
                trajectory_->at(t).update_lhs(dxi);

                // momentum
                previousState_[t] = beta_ * previousState_[t] - alpha * jacobian;

            }

            // ------------------------------------------------------------------------------------------------------
            // 3.X) NOT WORKING gradient decent with line search using the Backtracking algorithm (Nocedal p.37)
            // line search to satisfy t`he Wolfe conditions
            // (I)  f(x_k + a_k p_k) - f(x_k) <= c1 a_k Grad f_k p_k
            // (II) Grad f(x_k + a_k dx_k)'p_k >= c2 Grad f_k' dx_k
            if (solveMode_ == SolveMode::GRADIENT_DESCENT_BACKTRACKING)
            {
                double alpha = 0.05; // alpha \in (0,1)
                double updateError;
                Mat61 p_k = -jacobian, dxiOld = Mat61::Zero();
                bool backtrackingFlag;
                uint_t iters = 0;
                do
                {
                    Mat61 dxi = alpha * p_k;
                    trajectory_->at(t).update_lhs(-dxiOld);//XXX a little ugly...
                    trajectory_->at(t).update_lhs(dxi);
                    dxiOld = dxi;
                    // (I) calculate error at plane estimation for the current pose at time t
                    updateError = 0.0;
                    for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
                    {
                        updateError += it->second->get_error_incremental(t);
                    }
                    backtrackingFlag = (updateError <= initialError + c1_ * alpha * jacobian.dot(p_k));
                    alpha *= 0.7;
                    iters++;
                }while(!backtrackingFlag && iters < 20);
                //std::cout << "update error = " << updateError << ", initial error = " << initialError << ", and alpha = " << alpha << ", iter = "<< iters << std::endl;
            }

            // 3.X) NOT WORKING: BFGS, a first approach for alpha = 1 (no line search) or checking of Wolfe conditions.
            //         We assume the problem to be behave as a quadratic function, which is accurate on the manifold of SE3 but not for a sequence of
            //         poses. That makes the update state too agressive and the solution diverges
            if (solveMode_ == SolveMode::BFGS)
            {
                // dxi = - D * grad f  | for alpha = 1
                Mat61 dxi = - alpha_ * inverseHessian_[t]*jacobian ;
                trajectory_->at(t).update_lhs(dxi);
                // TODO Line search for updating alpha by satisfying Wolfe conditions
                // update inverseHessian Dk
                Mat61 y = jacobian/numberPoints - previousJacobian_[t];
                double rho = y.dot(dxi);
                //std::cout << "inverse hessian = " << inverseHessian_[t] << "\n gradient: "
                //          <<  jacobian << std::endl;
                inverseHessian_[t] = (Mat6::Identity() - dxi*y.transpose()/rho)*inverseHessian_[t]*(Mat6::Identity() - y*dxi.transpose()/rho) + dxi*dxi.transpose()/rho;
                previousJacobian_[t] = jacobian/numberPoints;
            }

            // 3.X) SR1
            // ------------------------------------------------------------------------------------------------------
        }
        solveIters++;
    }while(fabs(diffError) > 1e-4 && !singleIteration && solveIters < 1e4);
    //}while(!singleIteration && solveIters < 200);

    // correct for the first pose
    // The first pose should be T0 = I, but optimizition slighly perturns it, so we correct it here
    SE3 invFirstPose = trajectory_->at(0).inv();
    for (SE3 &pose: *trajectory_)
    {
        pose = invFirstPose * pose;
    }

    return solveIters;
}

uint_t PlaneRegistration::solve_interpolate(bool singleIteration)
{
    // iterative process, on convergence basis | error_k - error_k-1| < tol
    uint_t solveIters = 0;
    double previousError = 1e20, diffError = 10;
    do
    {
        // 1) calculate plane estimation given the current trajectory
        double  initialError = 0.0;
        for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
        {
            initialError += it->second->estimate_plane();
        }
        diffError = previousError - initialError;
        previousError = initialError;

        // 2) calculate Gradient = Jacobian^T. We maintain the nomenclature Jacobian for coherence on the project,
        //    but actually this Jacobian should be transposed.
        Mat61 jacobian = Mat61::Zero(), accumulatedJacobian = Mat61::Zero();
        double  numberPoints, tau = 1.0 / (double)(numberPoses_-1);
        for (uint_t t = 1 ; t < numberPoses_; ++t)
        {
            numberPoints = 0.0;
            jacobian.setZero();
            for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
            {
                jacobian += it->second->calculate_jacobian(t);
                numberPoints += it->second->get_number_points(t);
            }
            // XXX this could be changed to time stamps later
            accumulatedJacobian +=  (tau *  t  / numberPoints / numberPoses_) * jacobian;

        }
        // 3) update results Tf = exp(-dxi) * Tf (our convention, we expanded from the left)
        // 3-1)
        Mat61 dxi, xiFinal;
        if (solveMode_ == SolveMode::GRADIENT_DESCENT_NAIVE)
        {
            double alpha = alpha_;
            dxi = -alpha * accumulatedJacobian;
            //std::cout << "\nINterpolate jacobian : = " << accumulatedJacobian.transpose() << ", and increment update = " << dxi << std::endl;
        }
        // 3.4-B) Bengio's NAG: a modification to NAG as proposed in Bengio-2013. Fixed parameters
        //          1) momentum or velocity  v_k = beta_k-1 v _k-1 - alpha_k-1 Grad f (x_k-1)
        //          2) x_k+1 = x_k + beta_k+1 beta_k * v_k - (1 + beta_k+1)*alpha_k * Grad f(x_k)
        if (solveMode_ == SolveMode::BENGIOS_NAG)
        {
            double alpha = alpha_;
            double beta = beta_;
            // x update
            dxi = beta * beta * previousState_.back() - (1 + beta) * alpha * accumulatedJacobian;

            // momentum
            previousState_.back() = beta * previousState_.back() - alpha * accumulatedJacobian;
        }
        trajectory_->back().update_lhs(dxi);
        xiFinal = trajectory_->back().ln_vee();
        for (uint_t t = 1 ; t < numberPoses_-1; ++t)
        {
            dxi = tau * t * xiFinal;// SE3 does not like all derived classes TODO
            trajectory_->at(t) = SE3(dxi);
        }
        ++solveIters;
    }while(fabs(diffError) > 1e-4 && !singleIteration && solveIters < 1e4);
    return solveIters;
}

uint_t PlaneRegistration::solve_interpolate_hessian(bool singleIteration)
{
    // iterative process, on convergence basis | error_k - error_k-1| < tol
    // For now, only 1 iteration
    uint_t solveIters = 0;
    double previousError = 1e20, diffError = 10;

    do
    {

        // 1) calculate plane estimation given the current trajectory. Same as solve_interpolate
        double  initialError = 0.0;
        for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
        {
            initialError += it->second->estimate_plane();
        }
        diffError = previousError - initialError;
        previousError = initialError;

        // 2) calculate Gradient and Hessian
        Mat61 jacobian = Mat61::Zero(), accumulatedJacobian = Mat61::Zero();
        Mat6 hessian = Mat6::Zero(), accumulatedHessian = Mat6::Zero();
        double  tau = 1.0 / (double)(numberPoses_-1);
        for (uint_t t = 1 ; t < numberPoses_; ++t)
        {
            jacobian.setZero();
            for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
            {
                jacobian += it->second->calculate_jacobian(t);
                hessian += it->second->calculate_hessian(t);
            }
            // TODO this should be changed to time stamps later
            accumulatedJacobian +=  (tau *  t)  * jacobian;
            accumulatedHessian += (tau *  t) * hessian;
        }
        // 3) calculate update Tf = exp(-dxi) * Tf (our convention, we expanded from the left)
        // TODO this is an upper triangular matrix self adjoint matrix, inversion should take care of it
        Mat61 dxi = - accumulatedHessian.inverse() * jacobian;
        trajectory_->back().update_lhs(dxi);


        // 4) update full trajectory. Here we assume a full rank matrix TODO check for degenerate cases
        Mat61 xiFinal = trajectory_->back().ln_vee();
        for (uint_t t = 1 ; t < numberPoses_-1; ++t)
        {
            dxi = tau * t * xiFinal;// SE3 does not like all derived classes TODO
            trajectory_->at(t) = SE3(dxi);
        }
        solveIters++;
    }while(fabs(diffError) > 1e-4 && !singleIteration && solveIters < 1e4);
    return solveIters;
}

uint_t PlaneRegistration::solve_initialize()
{
    // TODO Maybe solve this as a plane-to-point alignment wrt T0
    // Initialize matrices of points
    MatX X(3,numberPlanes_), Y(3,numberPlanes_);

    // create points Y (from frame t =0), minimum 3 planes per pose
    uint_t t = 0;
    for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
    {
        it->second->calculate_all_matrices_S();
        Y.col(t) = it->second->get_mean_point(0);
        ++t;
    }

    // create points X, for t = 1, ... T, minimum 3 planes per pose
    for (t = 1; t < numberPoses_; ++t)
    {
        uint_t cont = 0;
        X.setZero();
        for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
        {
            X.col(cont) = it->second->get_mean_point(t);
            ++cont;
        }
        // Arun solver
        SE3 estimatedPose;
        if (!PCRegistration::arun(X,Y,estimatedPose))
            return 0;
        trajectory_->at(t) = estimatedPose;
    }


    // update current trajectory
    return 1;
}

double PlaneRegistration::get_current_error()
{
    double  currentError = 0.0;
    for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
        currentError += it->second->estimate_plane();
    return currentError;
}

void PlaneRegistration::add_plane(uint_t id, std::shared_ptr<Plane> &plane)
{
    plane->set_trajectory(trajectory_);
    planes_.emplace(id, plane);
}

double PlaneRegistration::calculate_poses_rmse(std::vector<SE3> & groundTruth) const
{
    assert(groundTruth.size() >= numberPoses_ && "PlaneRegistration::calculate_poses_rmse: number of poses from GT is incorrect\n");
    double rmse= 0.0;
    uint_t t = 0;
    for (auto &pose: groundTruth)
    {
        Mat61 dxi = (trajectory_->at(t) * pose.inv()).ln_vee();
        //std::cout << pose.ln_vee().transpose() << " and solution inv first pose \n" << (invFirstPose * trajectory_->at(t) * pose.inv()).ln_vee().transpose() <<std::endl;
        rmse += dxi.dot(dxi)/(double)numberPoses_;
        ++t;
    }
    return std::sqrt(rmse);
}

//XXX this can be a reference, who does this interface with pybinds?
std::vector<Mat31> PlaneRegistration::get_point_cloud(uint_t time)
{
	std::vector<Mat31> aggregated_pc;
	for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
    {
        std::vector<Mat31>& plane_pc = it->second->get_points(time);
        // aggregating elements
        aggregated_pc.insert(aggregated_pc.end(), plane_pc.begin(), plane_pc.end());
	}
	return aggregated_pc;
}

Mat4 PlaneRegistration::get_trajectory(uint_t time)
{
    assert(time < numberPoses_ && "CreatePoints::getPointCloud: temporal index larger than number of calculated poses\n");
    if (time < numberPoses_ )
        return trajectory_->at(time).T();
    return Mat4::Identity();
}

void PlaneRegistration::print(bool plotPlanes) const
{
    std::cout << "Printing plane registration data :"<< std::endl;
    for (SE3 &transf : *trajectory_)
        transf.print();
    if (plotPlanes)
    {
        for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
            it->second->print();
    }
}
