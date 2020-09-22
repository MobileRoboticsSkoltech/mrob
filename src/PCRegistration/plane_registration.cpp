/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * See LICENSE file in the root of the mrob library.
 *
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
#include <Eigen/Eigenvalues>
#include <iostream>


#include <chrono>

using namespace mrob;


PlaneRegistration::PlaneRegistration():
        numberPlanes_(0), numberPoses_(0),isSolved_(0), trajectory_(new std::vector<SE3>(8,SE3())),
        solveMode_(SolveMode::GRADIENT),
        c1_(1e-4), c2_(0.9), alpha_(0.75), beta_(0.1)
{
    // Optmizer does not establish the size for this matrices and thus it is required
    gradient_.resize(6);
    hessian_.resize(6,6);
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
            return optimize(NEWTON_RAPHSON);
        case SolveMode::GN_CLAMPED_HESSIAN:
            return solve_interpolate_hessian(singleIteration);
        case SolveMode::LM_SPHER:
            return optimize(LEVENBERG_MARQUARDT_SPHER);
        case SolveMode::LM_ELLIP:
            return optimize(LEVENBERG_MARQUARDT_ELLIP);
        default:
            return 0;
    }
}

uint_t PlaneRegistration::solve_interpolate_gradient(bool singleIteration)
{
    // This function is from the first implementation, is just kept for comparisons (but should not be used to solve the problem)
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

// TO BE DEPRECATED. Only used for clamped Hessian, and that is shown NOT to work.
uint_t PlaneRegistration::solve_interpolate_hessian(bool singleIteration)
{
    // iterative process, on convergence basis | error_k - error_k-1| < tol
    // For now, only 1 iteration
    solveIters_ = 0;
    double previousError = 1e20, diffError = 10;

    do
    {

        // 1) calculate plane estimation given the current trajectory. Same as solve_interpolate
        double  initialError = get_current_error();

        diffError = previousError - initialError;
        previousError = initialError;

        solveIters_++;
        std::cout << "current error iteration " << solveIters_ << " = "<< initialError << std::endl;

        // 2) calculate Gradient and Hessian
        Mat61 gradient = Mat61::Zero();
        Mat6 hessian = Mat6::Zero();
        gradient__.setZero();
        hessian__.setZero();
        double  tau = 1.0 / (double)(numberPoses_-1);
        for (uint_t t = 1 ; t < numberPoses_; ++t)
        {
            gradient.setZero();
            hessian.setZero();
            for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
            {
                gradient += it->second->calculate_gradient(t);
                hessian += it->second->calculate_hessian(t);
            }
            // TODO this should be changed to time stamps later
            gradient__ +=  (tau *  t)  * gradient;
            hessian__ += (tau *  t) * hessian.selfadjointView<Eigen::Upper>();
        }
        // 3) calculate update Tf = exp(-dxi) * Tf (our convention, we expanded from the left)
        Mat61 dxi;
        if (solveMode_ == SolveMode::GN_CLAMPED_HESSIAN)
        {
            // we clamp the vector spaces corresponding to negative eigenvals
            Mat6 pseudoInv = Mat6::Zero();
            Eigen::SelfAdjointEigenSolver<Mat6> eigs(hessian__);
            for (uint_t i = 0; i < 6 ; ++i)
            {
                if(eigs.eigenvalues()[i] > 1e-4 || true ) //TODO set tolerance
                {
                    std::cout << "POSITIVE. cos distance to grad = " << eigs.eigenvectors().col(i).dot(gradient)/gradient.norm()
                              << ", eigs = " << eigs.eigenvalues()[i] << std::endl;
                    pseudoInv += (1.0/eigs.eigenvalues()(i)) * eigs.eigenvectors().col(i) * eigs.eigenvectors().col(i).transpose();
                    // XXX why is this function not monotonically decreasing? this is annoying, but makes clamping a bad idea: LM!
                }
                else
                {
                    std::cout << "NEGATIVE. cos distance to grad = " << eigs.eigenvectors().col(i).dot(gradient)/gradient.norm()
                              << ", eigs = " << eigs.eigenvalues()[i] << std::endl;

                }
            }
            dxi = - pseudoInv * gradient__;
        }
        else
            dxi = - hessian__.inverse() * gradient__;
        trajectory_->back().update_lhs(dxi);


        // 4) update full trajectory. Here we assume a full rank matrix TODO check for degenerate cases
        Mat61 xiFinal = trajectory_->back().ln_vee();
        for (uint_t t = 1 ; t < numberPoses_-1; ++t)
        {
            dxi = tau * t * xiFinal;
            trajectory_->at(t) = SE3(dxi);
        }
    }while(fabs(diffError) > 1e-4 && !singleIteration && solveIters_ < 1e4);
    return solveIters_;
}


uint_t PlaneRegistration::solve_quaternion_plane()
{
    solveIters_ = 0;
    double previousError = 1e20, diffError = 10;

    // TODO create factor graph or call dense solver?


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


// resturns: [0]error, [1]iters, hessdet[2], conditioningNumber[3]
std::vector<double> PlaneRegistration::print_evaluate()
{
    std::vector<double> result(6,0.0);

    result[0] = get_current_error();
    result[1] = solveIters_;

    MatX allPlanes(numberPlanes_,4);
    MatX allNormals(numberPlanes_,3);
    uint_t i = 0;


    switch(solveMode_)
    {
        case SolveMode::GRADIENT:
        case SolveMode::GRADIENT_BENGIOS_NAG:
            hessian__.setZero();
            break;
        case SolveMode::GN_HESSIAN:
        case SolveMode::LM_SPHER:
        case SolveMode::LM_ELLIP:
            gradient__ = gradient_;
            hessian__ = hessian_;
    }

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
    std::cout << "current gradient \n" << gradient__ << std::endl;
    // XXX : NO, Orthogonolaity was not an issue
    //std::cout << "solution\n" << allPlanes << "\nOrthogonality between planes: \n" << allPlanes * allPlanes.transpose() <<
    //              "\n and det  = \n" << allPlanes.determinant() << std::endl;

    // XXX No, Orthogonality between normals
    //std::cout << "Orthogonality between normals: \n" << allNormals * allNormals.transpose() <<
    //             "\n and det = \n" << allNormals.determinant() << std::endl;

    // Hessian rank and eigen, look for negative vaps. Lasta hessina calculateds
    Eigen::EigenSolver<MatX> eigs(hessian__);
    std::cout << "eigen values are: \n" << eigs.eigenvalues() << std::endl;
    // Determinant of stacked normals
    std::cout << "det(Hessian) = \n" << hessian__ << std::endl;
    // Hessian conditioning number
    Eigen::JacobiSVD<Mat6> svd(hessian__, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // TODO remove
    //std::cout << "SVD decomposition : \n" << svd.singularValues() <<
    //              "\n vectors :\n" << svd.matrixU() <<
    //             "\n and conditioning number = " << svd.singularValues()(0)/svd.singularValues()(5) << std::endl;

    result[2] = hessian__.determinant();
    //TODO count this and optimze
    result[3] = svd.singularValues()(0)/svd.singularValues()(5);

    return result;
}

// From parent class Optimizer:
// TOOD for now replicated, later we will substitute
matData_t PlaneRegistration::calculate_error()
{
    return get_current_error();
}

void PlaneRegistration::calculate_gradient_hessian()
{
    Mat61 gradient = Mat61::Zero();
    Mat6 hessian = Mat6::Zero();
    gradient_.setZero();
    hessian_.setZero();
    double  tau = 1.0 / (double)(numberPoses_-1);
    for (uint_t t = 1 ; t < numberPoses_; ++t)
    {
        gradient.setZero();
        hessian.setZero();
        for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
        {
            gradient += it->second->calculate_gradient(t);
            hessian += it->second->calculate_hessian(t);
        }
        // TODO this should be changed to time stamps later
        gradient_ +=  (tau *  t)  * gradient;
        hessian_ += (tau *  t) * hessian.selfadjointView<Eigen::Upper>();
    }
}

void PlaneRegistration::update_state(const MatX1 &dx)
{
    trajectory_->back().update_lhs(dx);
    Mat61 xiFinal = trajectory_->back().ln_vee();
    double  tau = 1.0 / (double)(numberPoses_-1);
    Mat61 dxi;
    for (uint_t t = 1 ; t < numberPoses_-1; ++t)
    {
        dxi = tau * t * xiFinal;
        trajectory_->at(t) = SE3(dxi);
    }
}

void PlaneRegistration::bookkeep_state()
{
    bookept_trajectory_ = trajectory_->back();
}

void PlaneRegistration::update_state_from_bookkeep()
{
    trajectory_->back() = bookept_trajectory_;
    Mat61 xiFinal = bookept_trajectory_.ln_vee();
    double  tau = 1.0 / (double)(numberPoses_-1);
    Mat61 dxi;
    for (uint_t t = 1 ; t < numberPoses_-1; ++t)
    {
        dxi = tau * t * xiFinal;
        trajectory_->at(t) = SE3(dxi);
    }
    calculate_error();// planes get recalculated, which is a requisite for later
    //(this class construction, in general grad should be self-contained...)
}

