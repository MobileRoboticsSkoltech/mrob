/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * planeRegistration.cpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include "mrob/plane_registration.hpp"
#include <iostream>

using namespace mrob;


PlaneRegistration::PlaneRegistration():
        numberPlanes_(0), numberPoses_(0),isSolved_(0), trajectory_(new std::vector<SE3>(8,SE3())),
        solveMode_(SolveMode::BFGS),
        c1_(1e-4), c2_(0.9)
{
    solveMode_ = SolveMode::GRADIENT_DESCENT_BACKTRACKING;
    //solveMode_ = SolveMode::GRADIENT_DESCENT_NAIVE;
}

PlaneRegistration::PlaneRegistration(uint_t numberPlanes, uint_t numberPoses):
        numberPlanes_(numberPlanes), numberPoses_(numberPoses),isSolved_(0),
        trajMode_(TrajectoryMode::SEQUENCE), trajectory_(new std::vector<SE3>(numberPoses, SE3())),
        solveMode_(SolveMode::BFGS),
        c1_(1e-4), c2_(0.9)
{
    planes_.reserve(numberPlanes);
    inverseHessian_.resize(numberPoses, 1e-3 * Mat6::Identity());
    previousJacobian_.resize(numberPoses, Mat61::Zero());
}

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
}

uint_t PlaneRegistration::solve()
{
    // Initializitaion
    //inverseHessian_.clear();
    //inverseHessian_.resize(numberPoses, Mat6::Identity());
    //Mat61 previousJacobian = Mat61::Zero();


    // TODO iterative process, on what convergence basis?
    uint_t solveIters = 0;
    double previousError = 1e20, diffError = 10;
    while(diffError > 1e-4)
    {

        // 1) calculate plane estimation given the current trajectory
        double  initialError = 0.0;
        for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
        {
                it->second->estimate_plane();
                initialError += it->second->get_error();
        }
        diffError = previousError - initialError;
        previousError = initialError;

        // 2) calculate Jacobians
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
            }

            // 3) update results Ti = exp(-dxi) * Ti (our convention, we expanded from the left)
            // 3.1) first atempt: gradient decent with fixed step upgrade alpha = 1/N
            if (solveMode_ == SolveMode::GRADIENT_DESCENT_NAIVE)
            {
                Mat61 dxi = -jacobian/(1.5*numberPoints);// dxi = alpha * p_k = alpha *(-Grad f)
                std::cout << "jacobian : = " << jacobian.norm() << std::endl;
                trajectory_->at(t).update(dxi);
            }

        // line search to satisfy t`he Wolfe conditions
        // (I)  f(x_k + a_k p_k) - f(x_k) <= c1 a_k Grad f_k p_k
        // (II) Grad f(x_k + a_k dx_k)'p_k >= c2 Grad f_k' dx_k
            // 3.2) gradient decent with line search using the Backtracking algorithm (Nocedal p.37)
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
                    trajectory_->at(t).update(-dxiOld);//XXX a little ugly...
                    trajectory_->at(t).update(dxi);
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

            // 3.3) BFGS, a first approach for alpha = 1 (no line search) or checking of Wolfe conditions.
            //         We assume the problem to be behave as a quadratic function, which is accurate on the manifold of SE3
            if (solveMode_ == SolveMode::BFGS)
            {
                // dxi = - D * grad f  | for alpha = 1
                Mat61 dxi = - 0.5 * inverseHessian_[t]*jacobian ;
                trajectory_->at(t).update(dxi);
                // TODO Line search for updating alpha by satisfying Wolfe conditions
                // update inverseHessian Dk
                Mat61 y = jacobian - previousJacobian_[t];
                double rho = y.dot(dxi);
                std::cout << "inverse hessian = " << inverseHessian_[t] << std::endl;
                inverseHessian_[t] = (Mat6::Identity() - dxi*y.transpose()/rho)*inverseHessian_[t]*(Mat6::Identity() - y*dxi.transpose()/rho) + dxi*dxi.transpose()/rho;
                previousJacobian_[t] = jacobian;
            }

            // 3.3) SR1
        }
        std::cout << "error =                                                 " << initialError << std::endl;
        solveIters++;
    }

    return solveIters;
}

void PlaneRegistration::add_plane(uint_t id, std::shared_ptr<Plane> &plane)
{
    plane->set_trajectory(trajectory_);
    planes_.emplace(id, plane);
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
