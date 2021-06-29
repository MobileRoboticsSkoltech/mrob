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
        numberPlanes_(0), numberPoses_(0),numberPoints_(0),isSolved_(0), trajectory_(new std::vector<SE3>(8,SE3())),
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

// XXX: is this function used??
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

    numberPoints_ = 0;
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
    uint_t iters = 0;
    initial_error_ = get_current_error();

    // time profiling
    time_profiles_.reset();
    switch(mode)
    {
        case SolveMode::INITIALIZE:
            return solve_initialize();
        case SolveMode::GRADIENT_ALL_POSES:
            time_profiles_.start();
            solve_gradient_all_poses();
            time_profiles_.stop();
            break;
        case SolveMode::GRADIENT_BENGIOS_NAG:
            time_profiles_.start();
            solve_interpolate_gradient(singleIteration);
            time_profiles_.stop();
            break;
        case SolveMode::GN_HESSIAN:
            time_profiles_.start();
            solveIters_ = Optimizer::solve(NEWTON_RAPHSON);
            time_profiles_.stop();
            break;
        case SolveMode::GN_CLAMPED_HESSIAN:
            time_profiles_.start();
            //solveIters_ = solve_interpolate_hessian(singleIteration);//deprecated
            time_profiles_.stop();
            break;
        case SolveMode::LM_SPHER:
            time_profiles_.start();
            solveIters_ = Optimizer::solve(LEVENBERG_MARQUARDT_SPHER,100,1e-2);
            time_profiles_.stop();
            break;
        case SolveMode::LM_ELLIP:
            time_profiles_.start();
            solveIters_ = Optimizer::solve(LEVENBERG_MARQUARDT_ELLIP,100,1e-2);
            time_profiles_.stop();
            break;
        default:
            return 0;
    }
    return iters;
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

        //std::cout << "current error iteration " << solveIters_ << " = "<< initialError << std::endl;

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

uint_t PlaneRegistration::solve_gradient_all_poses(bool singleIteration)
{
    // This function is from the first implementation too, is just kept for comparisons in a full traj optimization
    solveIters_ = 0;
    double previousError = 1e20, diffError = 10;
    do
    {
        // 1) calculate plane estimation given the current trajectory
        double  initialError = get_current_error();
        diffError = previousError - initialError;
        previousError = initialError;

        //std::cout << "current error iteration " << solveIters_ << " = "<< initialError << std::endl;

        // 2) calculate Gradient
        MatX1 jacobian = MatX1::Zero(numberPoses_*6);
        double  numberPoints; //, tau = 1.0 / (double)(numberPoses_-1);
        for (uint_t t = 1 ; t < numberPoses_; ++t)
        {
            numberPoints = 0.0;
            for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
            {
                jacobian.segment<6>(t*6) += it->second->calculate_gradient(t);
                numberPoints += it->second->get_number_points(t);
            }

        }
        // 3) update results Tf = exp(-dxi) * Tf (our convention, we expanded from the left)
        // 3-1)
        Mat61 dxi;
        for (uint_t t = 1 ; t < numberPoses_; ++t)
        {
            Mat61 dxi = -(alpha_ / numberPoints / numberPoses_ )*jacobian.segment<6>(t*6);
            trajectory_->at(t).update_lhs(dxi);
        }
        ++solveIters_;
    }while(fabs(diffError) > 1e-4 && !singleIteration && solveIters_ < 1e4);
    return solveIters_;
}

uint_t PlaneRegistration::solve_quaternion_plane()
{
    solveIters_ = 0;

    // TODO create factor graph or call dense solver?


    return solveIters_;
}

uint_t PlaneRegistration::solve_initialize()
{
    // Initialize matrices of , all points in the Y matrix
    std::vector<Mat31> Y_points_all;
    Y_points_all.reserve(numberPlanes_);

    // update data structure for init. All point in the fist pose
    for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
    {
        it->second->calculate_all_matrices_S();
        if (it->second->get_number_points(0) > 0 )
            Y_points_all.push_back(it->second->get_mean_point(0));
    }

    uint_t t = 0;
    // create points Y,X, for t = 1, ... T, minimum 3 planes per pose
    std::vector<Mat31> X_points, Y_points;
    X_points.reserve(numberPlanes_);
    Y_points.reserve(numberPlanes_);
    uint_t number_points;

    for (t = 1; t < numberPoses_; ++t)
    {
        X_points.clear();
        Y_points.clear();
        number_points = 0;
        uint_t cont =0;
        for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
        {
            if (it->second->get_number_points(t) > 0 )
            {
                X_points.push_back(it->second->get_mean_point(t));
                Y_points.push_back(Y_points_all[cont]);
                number_points++;
            }
            cont++;
        }
        // Create the matrices X,Y
        MatX X(number_points,3), Y(number_points,3);
        for (uint_t i = 0; i < number_points; ++i)
        {
            X.row(i) = X_points[i];
            Y.row(i) = Y_points[i];
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

void PlaneRegistration::set_last_pose(SE3 &last)
{
    Mat61 xiFinal = last.ln_vee();
    double  tau = 1.0 / (double)(numberPoses_-1);
    Mat61 dxi;
    for (uint_t t = 1 ; t < numberPoses_; ++t)
    {
        dxi = tau * t * xiFinal;
        trajectory_->at(t) = SE3(dxi);
    }
}

void PlaneRegistration::add_plane(uint_t id, std::shared_ptr<Plane> &plane)
{
    plane->set_trajectory(trajectory_);
    planes_.emplace(id, plane);
}

void PlaneRegistration::add_new_plane(uint_t id)
{
    std::shared_ptr<Plane> plane(new Plane(numberPoses_));
    plane->set_trajectory(trajectory_);
    planes_.emplace(id, plane);
}

void PlaneRegistration::plane_push_back_point(uint_t id, uint_t t, Mat31 &point)
{
    planes_.at(id)->push_back_point(point,t);
}

uint_t PlaneRegistration::calculate_total_number_points()
{
    numberPoints_ = 0;
    for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
        numberPoints_ += it->second->get_total_number_points();
    return numberPoints_;
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

SE3 PlaneRegistration::get_trajectory(uint_t time)
{
    assert(time < numberPoses_ && "CreatePoints::getPointCloud: temporal index larger than number of calculated poses\n");
    if (time < numberPoses_ )
        return trajectory_->at(time);
    return SE3();
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


//         Nplanes[0], Nposes[1], Npoints[2], iters[3],  process-time[4],
//         ini_error[5] error[6], eigenvalues[7-12]
std::vector<double> PlaneRegistration::print_evaluate()
{
    std::vector<double> result(13,0.0);

    result[0] = numberPlanes_;
    result[1] = numberPoses_;
    calculate_total_number_points();
    result[2] = numberPoints_;
    result[3] = solveIters_;
    double k = 1.0;//numberPoints_;
    result[4] = time_profiles_.total_time();
    result[5] = initial_error_/k;
    result[6] = get_current_error()/k; //TODO this does not work for GN (but yes to GN ini...)

    MatX allPlanes(numberPlanes_,4);
    MatX allNormals(numberPlanes_,3);
    uint_t i = 0;


    switch(solveMode_)
    {
        case SolveMode::GRADIENT:
        case SolveMode::GRADIENT_BENGIOS_NAG:
        case SolveMode::GRADIENT_ALL_POSES:
            hessian__.setZero();
            break;
        case SolveMode::GN_HESSIAN:
        case SolveMode::LM_SPHER:
        case SolveMode::LM_ELLIP:
            gradient__ = gradient_;
            hessian__ = hessian_;
    }

    // Normals on planes, check for rank
    for (auto &&plane : planes_)
    {
        Mat41 pi = plane.second->get_plane();
        //std::cout << "plane : \n" << pi << std::endl;
        allPlanes.row(i) = pi;
        allNormals.row(i) = pi.head(3)/(pi.head(3).norm());
        ++i;
    }
    // Orthogonality between planes (4 dim)
    //std::cout << "current gradient \n" << gradient__ << std::endl;
    // XXX : NO, Orthogonolaity was not an issue
    //std::cout << "solution\n" << allPlanes << "\nOrthogonality between planes: \n" << allPlanes * allPlanes.transpose() <<
    //              "\n and det  = \n" << allPlanes.determinant() << std::endl;

    // XXX No, Orthogonality between normals
    //std::cout << "Orthogonality between normals: \n" << allNormals * allNormals.transpose() <<
    //             "\n and det = \n" << allNormals.determinant() << std::endl;

    // Hessian rank and eigen, look for negative vaps. Lasta hessina calculateds
    Eigen::SelfAdjointEigenSolver<MatX> eigs(hessian__);
    //std::cout << "eigen values are: \n" << eigs.eigenvalues() << std::endl;
    // Determinant of stacked normals
    //std::cout << "det(Hessian) = \n" << hessian__ << std::endl;

    for (uint_t i = 0; i < 6; ++i)
        result[7+i] = eigs.eigenvalues()(i);

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
    get_current_error();//this recalculates the current planes estimation, required for LM proper undoing.
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

