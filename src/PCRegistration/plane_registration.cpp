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
        solveMode_(SolveMode::GRADIENT),
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

    previousState_.clear();
    previousState_.resize(numberPoses, Mat61::Zero());
}


void PlaneRegistration::reset_solution()
{
    // trajectory is reset
    trajectory_->clear();
    trajectory_->resize(numberPoses_, SE3());
    previousState_.clear();
    previousState_.resize(numberPoses_, Mat61::Zero());
}


uint_t PlaneRegistration::solve(SolveMode mode, bool singleIteration)
{
    // just in case some methods, such as gradient, has several modes
    solveMode_ = mode;
    switch(mode)
    {
        case SolveMode::INITIALIZE:
            return solve_initialize();
        case SolveMode::GRADIENT:
        case SolveMode::GRADIENT_BENGIOS_NAG:
            return solve_interpolate_gradient(singleIteration);
        case SolveMode::GN_HESSIAN:
            return solve_interpolate_hessian(singleIteration);
        case SolveMode::GN_CLAMPED_HESSIAN:
        case SolveMode::LM_HESSIAN:
        case SolveMode::LM_CLAMPED_HESSIAN:
        default:
            return 0;
    }
}

uint_t PlaneRegistration::solve_interpolate_gradient(bool singleIteration)
{
    // iterative process, on convergence basis | error_k - error_k-1| < tol
    solveIters_ = 0;
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

        std::cout << "current error iteration " << solveIters_ << " = "<< initialError << std::endl;

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
                jacobian += it->second->calculate_gradient(t);
                numberPoints += it->second->get_number_points(t);
            }
            // XXX this could be changed to time stamps later
            accumulatedJacobian +=  (tau *  t  / numberPoints / numberPoses_) * jacobian;

        }
        // 3) update results Tf = exp(-dxi) * Tf (our convention, we expanded from the left)
        // 3-1)
        Mat61 dxi, xiFinal;
        if (solveMode_ == SolveMode::GRADIENT)
        {
            double alpha = alpha_;
            dxi = -alpha * accumulatedJacobian;
            //std::cout << "\nINterpolate jacobian : = " << accumulatedJacobian.transpose() << ", and increment update = " << dxi << std::endl;
        }
        // 3.4-B) Bengio's NAG: a modification to NAG as proposed in Bengio-2013. Fixed parameters
        //          1) momentum or velocity  v_k = beta_k-1 v _k-1 - alpha_k-1 Grad f (x_k-1)
        //          2) x_k+1 = x_k + beta_k+1 beta_k * v_k - (1 + beta_k+1)*alpha_k * Grad f(x_k)
        if (solveMode_ == SolveMode::GRADIENT_BENGIOS_NAG)
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
        ++solveIters_;
    }while(fabs(diffError) > 1e-4 && !singleIteration && solveIters_ < 1e4);
    return solveIters_;
}

uint_t PlaneRegistration::solve_interpolate_hessian(bool singleIteration)
{
    // iterative process, on convergence basis | error_k - error_k-1| < tol
    // For now, only 1 iteration
    solveIters_ = 0;
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

        std::cout << "current error iteration " << solveIters_ << " = "<< initialError << std::endl;

        // 2) calculate Gradient and Hessian
        Mat61 gradient = Mat61::Zero();
        Mat6 hessian = Mat6::Zero();
        gradient_.setZero();
        hessian_.setZero();
        double  tau = 1.0 / (double)(numberPoses_-1);
        for (uint_t t = 1 ; t < numberPoses_; ++t)
        {
            gradient.setZero();
            for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
            {
                gradient += it->second->calculate_gradient(t);
                hessian += it->second->calculate_hessian(t);
            }
            // TODO this should be changed to time stamps later
            gradient_ +=  (tau *  t)  * gradient;
            hessian_ += (tau *  t) * hessian.selfadjointView<Eigen::Upper>();
        }
        // 3) calculate update Tf = exp(-dxi) * Tf (our convention, we expanded from the left)
        // TODO this is an upper triangular matrix self adjoint matrix, inversion should take care of it
        Mat61 dxi = - hessian_.inverse() * gradient;
        trajectory_->back().update_lhs(dxi);


        // 4) update full trajectory. Here we assume a full rank matrix TODO check for degenerate cases
        Mat61 xiFinal = trajectory_->back().ln_vee();
        for (uint_t t = 1 ; t < numberPoses_-1; ++t)
        {
            dxi = tau * t * xiFinal;
            trajectory_->at(t) = SE3(dxi);
        }
        solveIters_++;
    }while(fabs(diffError) > 1e-4 && !singleIteration && solveIters_ < 1e4);
    return solveIters_;
}


uint_t PlaneRegistration::solve_quaternion_plane()
{
    solveIters_ = 0;
    //double previousError = 1e20, diffError = 10;

    return solveIters_;
}

uint_t PlaneRegistration::solve_initialize()
{
    // TODO Maybe solve this as a plane-to-point alignment wrt T0
    // Initialize matrices of points
    MatX X(numberPlanes_,3), Y(numberPlanes_,3);

    // create points Y (from frame t =0), minimum 3 planes per pose
    uint_t t = 0;
    for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
    {
        it->second->calculate_all_matrices_S();
        Y.row(t) = it->second->get_mean_point(0);
        ++t;
    }

    // create points X, for t = 1, ... T, minimum 3 planes per pose
    for (t = 1; t < numberPoses_; ++t)
    {
        uint_t cont = 0;
        X.setZero();
        for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
        {
            X.row(cont) = it->second->get_mean_point(t);
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

double PlaneRegistration::get_current_error() const
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

#include <Eigen/Eigenvalues>

// resturns: [0]error, [1]iters, hessdet[2], conditioningNumber[3]
std::vector<double> PlaneRegistration::print_evaluate() const
{
    std::vector<double> result(6,0.0);

    result[0] = get_current_error();
    result[1] = solveIters_;

    MatX allPlanes(numberPlanes_,4);
    MatX allNormals(numberPlanes_,3);
    uint_t i = 0;
    // Normals on planes, check for rank
    for (auto plane : planes_)
    {
        Mat41 pi = plane.second->get_plane();
        //std::cout << "plane : \n" << pi << std::endl;
        allPlanes.row(i) = pi;
        allNormals.row(i) = pi.head(3)/(pi.head(3).norm());
        ++i;
    }
    // Orthogonality between planes (4 dim)
    std::cout << "current gradient \n" << gradient_ << std::endl;
    // XXX : NO, Orthogonolaity was not an issue
    //std::cout << "solution\n" << allPlanes << "\nOrthogonality between planes: \n" << allPlanes * allPlanes.transpose() <<
    //              "\n and det  = \n" << allPlanes.determinant() << std::endl;

    // XXX No, Orthogonality between normals
    //std::cout << "Orthogonality between normals: \n" << allNormals * allNormals.transpose() <<
    //             "\n and det = \n" << allNormals.determinant() << std::endl;

    // Hessian rank and eigen, look for negative vaps. Lasta hessina calculateds
    Eigen::EigenSolver<MatX> eigs(hessian_);
    std::cout << "eigen values are: \n" << eigs.eigenvalues() << std::endl;
    // Determinant of stacked normals
    std::cout << "det(Hessian) = \n" << hessian_ << std::endl;
    // Hessian conditioning number
    Eigen::JacobiSVD<Mat6> svd(hessian_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::cout << "SVD decomposition : \n" << svd.singularValues() <<
                  "\n vectors :\n" << svd.matrixU() <<
                 "\n and conditioning number = " << svd.singularValues()(0)/svd.singularValues()(5) << std::endl;

    result[2] = hessian_.determinant();
    //TODO count this and optimze
    result[3] = svd.singularValues()(0)/svd.singularValues()(5);

    return result;
}
