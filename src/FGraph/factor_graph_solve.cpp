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
 * factor_graph_solve.cpp
 *
 *  Created on: Mar 23, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include "mrob/factor_graph_solve.hpp"
//#include "mrob/CustomCholesky.hpp"

#include <iostream>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseQR>

using namespace mrob;
using namespace std;
using namespace Eigen;


FGraphSolve::FGraphSolve(matrixMethod method):
	FGraph(), matrixMethod_(method), N_(0), M_(0),
	lambda_(1e-6), solutionTolerance_(1e-2)
{

}

FGraphSolve::~FGraphSolve() = default;


void FGraphSolve::solve(optimMethod method, uint_t maxIters, matData_t lambda, matData_t solutionTolerance)
{
    /**
     * 2800 2D nodes on M3500
     * Time profile :13.902 % build Adjacency matrix,
     *               34.344 % build Information,
     *               48.0506 % build Cholesky,
     *               2.3075 % solve forward and back substitution,
     *               1.3959 % update values,
     *
     */
    lambda_ = lambda;
    solutionTolerance_ = solutionTolerance;
    time_profiles_.reset();
    optimMethod_ = method;

    // Optimization
    switch(method)
    {
      case GN:
        this->optimize_gauss_newton();// false => lambda = 0
        this->update_nodes();
        break;
      case LM:
      case LM_ELLIPS:
        this->optimize_levenberg_marquardt(maxIters);
        break;
      default:
        assert(0 && "FGraphSolve:: optimization method unknown");
    }



    // TODO add variable verbose to output times
    if (1)
        time_profiles_.print();
}

void FGraphSolve::build_problem(bool useLambda)
{

    // 1) Adjacency matrix A, it has to
    //    linearize and calculate the Jacobians and required matrices
    time_profiles_.start();
    this->build_adjacency();
    time_profiles_.stop("Adjacency");

    if (eigen_factors_.size()>0)
    {
        time_profiles_.start();
        this->build_info_EF();
        time_profiles_.stop("EFs Jacobian and Hessian");
    }

    // 1.2) builds specifically the information
    switch(matrixMethod_)
    {
      case ADJ:
        time_profiles_.start();
        this->build_info_adjacency();
        time_profiles_.stop("Info Adjacency");
        break;
      case SCHUR:
      default:
        assert(0 && "FGraphSolve: method not implemented");
    }

    // 1.3) (Optional) Eigen Factors

    // Structure for LM and dampening GN-based methods
    if (useLambda)
    {
        diagL_ = L_.diagonal();
    }
}

void FGraphSolve::optimize_gauss_newton(bool useLambda)
{
    // requires a Column-storage matrix
    SimplicialLDLT<SMatCol,Lower, AMDOrdering<SMatCol::StorageIndex>> cholesky;

    this->build_problem(useLambda);

    // compute cholesky solution
    time_profiles_.start();
    if (useLambda)
    {
        for (uint_t n = 0 ; n < N_; ++n)
        {
            if (optimMethod_ == LM)
                L_.coeffRef(n,n) = lambda_ + diagL_(n);//Spherical
            else
                L_.coeffRef(n,n) = (1.0 + lambda_)*diagL_(n);//Elipsoid
        }
    }
    cholesky.compute(L_);
    time_profiles_.stop("Gauss Newton create Cholesky");
    time_profiles_.start();
    dx_ = cholesky.solve(b_);
    time_profiles_.stop("Gauss Newton solve Cholesky");

}

uint_t FGraphSolve::optimize_levenberg_marquardt(uint_t maxIters)
{
    //SimplicialLDLT<SMatCol,Lower, AMDOrdering<SMatCol::StorageIndex>> cholesky;


    // LM trust region as described in Bertsekas (p.105)

    // 0) parameter initialization
    lambda_ = 1e-5;
    // sigma reference to the fidelity of the model at the proposed solution \in [0,1]
    matData_t sigma1(0.25), sigma2(0.8);// 0 < sigma1 < sigma2 < 1
    matData_t beta1(2.0), beta2(0.25); // lambda updates multiplier values, beta1 > 1 > beta2 >0
    //matData_t lambdaMax, lambdaMin; // XXX lower bound unnecessary

    matData_t currentChi2, deltaChi2, modelFidelity;
    uint_t iter = 0;

    do{
        iter++;
        // 1) solve subproblem and current error
        this->optimize_gauss_newton(true);// Test if solved anything? no nans
        currentChi2 = this->chi2(false);// TODO residuals don't need to be calculated again (see optimizer.cpp)
        this->synchronize_nodes_auxiliary_state();// book-keeps states to undo updates
        this->update_nodes();


        // 1.2) Check for convergence, needs update and re-evaluaiton of errors
        deltaChi2 = currentChi2 - this->chi2(true);
        std::cout << "\nFGraphSolve::optimize_levenberg_marquardt: iteration "
                  << iter << " lambda = " << lambda_ << ", error " << currentChi2
                  << ", and delta = " << deltaChi2
                  << std::endl;
        if (deltaChi2 < 0)
        {
            // proposed dx did not improve, repeat 1) and reduce area of optimization = increase lambda
            lambda_ *= beta1;
            this->synchronize_nodes_state();
            continue;
        }

        // 1.3) check for convergence
        if (deltaChi2 < solutionTolerance_)
            return iter;


        // 2) Fidelity of the quadratized model vs non-linear chi2 evaluation.
        // f = chi2(x_k) - chi2(x_k + dx)
        //     chi2(x_k) - m_k(dx)
        // where m_k is the quadratized model = ||r||^2 - dx'*J' r + 0.5 dx'(J'J + lambda*D2)dx
        modelFidelity = deltaChi2 / (dx_.dot(b_) - 0.5*dx_.dot(L_* dx_));
        std::cout << "model fidelity = " << modelFidelity << " and m_k = " << dx_.dot(b_) << std::endl;

        //3) update lambda
        if (modelFidelity < sigma1)
            lambda_ *= beta1;
        if (modelFidelity > sigma2)
            lambda_ *= beta2;


    } while (iter < maxIters);

    // output
    std::cout << "FGraphSolve::optimize_levenberg_marquardt: failed to converge after "
              << iter << " iterations and error " << currentChi2
              << ", and delta = " << deltaChi2
              << std::endl;
    return 0; //

}

void FGraphSolve::build_adjacency()
{
    // Check for consistency. With 0 observations the problem cannot be built.
    assert(obsDim_ >0 && "FGraphSolve::build_adjacency: Zeros observations, at least add one anchor factor");
    if (obsDim_ == 0)
        return;

    // 0) resize properly matrices (if needed)
    r_.resize(obsDim_,1);//dense vector TODO is it better to reserve and push_back??
    A_.resize(obsDim_, stateDim_);//Sparse matrix clears data, but keeps the prev reserved space
    W_.resize(obsDim_, obsDim_);//TODO should we reinitialize this all the time? an incremental should be fairly easy

    // 1) create the vector's structures
    std::deque<std::shared_ptr<Factor> >* factors;
    std::deque<std::shared_ptr<Node> >* nodes;
    // TODO: optimizing subgraph is not an option now, but we maintain generality.
    factors = &factors_;
    nodes = &nodes_;

    // 2) vector structure to bookkeep the starting Nodes indices inside A

    // 2.2) Node indexes bookeept
    indNodesMatrix_.clear();
    indNodesMatrix_.reserve(nodes->size());

    // XXX: this structure ONLY works because we use all nodes. If we were using a subset of them (l227)
    // Then this ordered approach for ids {0,..,N} would not work!
    N_ = 0;
    for (size_t i = 0; i < nodes->size(); ++i)
    {
        // calculate the indices to access
        uint_t dim = (*nodes)[i]->get_dim();
        indNodesMatrix_.push_back(N_);
        N_ += dim;
    }
    assert(N_ == stateDim_ && "FGraphSolve::buildAdjacency: State Dimensions are not coincident\n");

    // 3) Evaluate every factor given the current state and bookeeping of Factor indices
    std::vector<uint_t> reservationA;
    reservationA.reserve( obsDim_ );
    std::vector<uint_t> reservationW;
    reservationW.reserve( obsDim_ );
    std::vector<uint_t> indFactorsMatrix;
    indFactorsMatrix.reserve(factors->size());
    M_ = 0;
    for (uint_t i = 0; i < factors->size(); ++i)
    {
        auto f = (*factors)[i];
        f->evaluate_residuals();
        f->evaluate_jacobians();
        f->evaluate_chi2();

        // calculate dimensions for reservation and bookeping vector
        uint_t dim = f->get_dim();
        uint_t allDim = f->get_all_nodes_dim();
        for (uint_t j = 0; j < dim; ++j)
        {
            reservationA.push_back(allDim);
            reservationW.push_back(dim-j);//XXX this might be allocating more elements than necessary, check
        }
        indFactorsMatrix.push_back(M_);
        M_ += dim;
    }
    assert(M_ == obsDim_ && "FGraphSolve::buildAdjacency: Observation dimensions are not coincident\n");
    A_.reserve(reservationA); //Exact allocation for elements.
    W_.reserve(reservationW); //same


    // XXX This could be subject to parallelization, maybe on two steps: eval + build
    for (uint_t i = 0; i < factors->size(); ++i)
    {
        auto f = (*factors)[i];

        // 4) Get the calculated residual
        r_.block(indFactorsMatrix[i], 0, f->get_dim(), 1) <<  f->get_residual();

        // 5) build Adjacency matrix as a composition of rows
        // 5.1) Get the number of nodes involved. It is a vector of nodes
        auto neighNodes = f->get_neighbour_nodes();
        // Iterates over the Jacobian row
        for (uint_t l=0; l < f->get_dim() ; ++l)
        {
            uint_t totalK = 0;
            // Iterates over the number of neighbour Nodes (ordered by construction)
            for (uint_t j=0; j < neighNodes->size(); ++j)
            {
                uint_t indNode = (*neighNodes)[j]->get_id();
                uint_t dimNode = (*neighNodes)[j]->get_dim();
                for(uint_t k = 0; k < dimNode; ++k)
                {
                    // order according to the permutation vector
                    uint_t iRow = indFactorsMatrix[i] + l;
                    uint_t iCol = indNodesMatrix_[indNode] + k;
                    // This is an ordered insertion
                    A_.insert(iRow,iCol) = f->get_jacobian()(l, k + totalK);
                }
                totalK += dimNode;
            }
        }


        // 5) Get information matrix for every factor
        // TODO for robust factors, here is where the robust weights should be applied
        matData_t robust_weight = 1.0;
        for (uint_t l = 0; l < f->get_dim(); ++l)
        {
            // only iterates over the upper triangular part
            for (uint_t k = l; k < f->get_dim(); ++k)
            {
                uint_t iRow = indFactorsMatrix[i] + l;
                uint_t iCol = indFactorsMatrix[i] + k;
                robust_weight = f->evaluate_robust_weight(std::sqrt(f->get_chi2()));
                W_.insert(iRow,iCol) = robust_weight * f->get_information_matrix()(l,k);
            }
        }
    } //end factors loop


}

void FGraphSolve::build_info_adjacency()
{
    /**
     * L_ dx = b_ corresponds to the normal equation A'*W*A dx = A'*W*r
     * only store the lower part of the information matrix (symmetric)
     *
     * XXX: In terms of speed, using the selfadjointview does not improve,
     * Eigen stores a temporary object and then copy only the upper part.
     *
     */
    L_ = (A_.transpose() * W_.selfadjointView<Eigen::Upper>() * A_);
    b_ = A_.transpose() * W_.selfadjointView<Eigen::Upper>() * r_;

    // If any EF, we should combine both solutions
    if (eigen_factors_.size() > 0 )
    {
        L_ += hessianEF_.selfadjointView<Eigen::Upper>();
        b_ += gradientEF_;
    }
}


void FGraphSolve::build_info_EF()
{
    gradientEF_.resize(stateDim_,1);
    gradientEF_.setZero();
    std::vector<Triplet> hessianData;
    // XXX if EF ever connected a node that is not 6D, then this will not hold.
    hessianData.reserve(eigen_factors_.size()*21);//For each EF we reserve the uppder triangular view => 6+5+..+1  = 21
    // It assumes L_ has been created, and requires at least 1 observation (achnor)
    for (size_t id = 0; id < eigen_factors_.size(); ++id)
    {
        auto f = eigen_factors_[id];
        f->evaluate_residuals();
        f->evaluate_jacobians();//and Hessian
        f->evaluate_chi2();
        auto neighNodes = f->get_neighbour_nodes();
        for (auto node : *neighNodes)
        {
            uint_t indNode = node->get_id();
            // Updating Jacobian, b should has been previously calculated
            Mat61 J = f->get_jacobian(indNode);
            gradientEF_.block<6,1>(indNodesMatrix_[indNode],0) += J;//TODO robust weight would go here
            //std::cout << "Jacobian = " << J.transpose() << std::endl;

            // Updating the Hessian
            Mat6 H = f->get_hessian(indNode);
            //std::cout << "Hessian = " << H << std::endl;
            uint_t startingIndex = indNodesMatrix_[indNode];//XXX this should be change (someday) to a proper table for any ordering
            // XXX if EF ever connected a node that is not 6D, then this will not hold.
            for (uint_t i = 0; i < 6; i++)
            {
                for (uint_t j = i; j<6; j++)
                {
                    // convert the hessian to triplets
                    hessianData.emplace_back(Triplet(startingIndex+ i, startingIndex+ j, H(i,j)));
                }
            }
        }
    }

    // create a Upper-view sparse matrix from the triplets:
    hessianEF_.resize(stateDim_,stateDim_);
    hessianEF_.setFromTriplets(hessianData.begin(), hessianData.end());
}

matData_t FGraphSolve::chi2(bool evaluateResidualsFlag)
{
    matData_t totalChi2 = 0.0;
    for (uint_t i = 0; i < factors_.size(); ++i)
    {
        auto f = factors_[i];
        if (evaluateResidualsFlag)
        {
            f->evaluate_residuals();
            f->evaluate_chi2();
        }
        totalChi2 += f->get_chi2();
    }

    for (auto &ef : eigen_factors_)
    {
        if (evaluateResidualsFlag)
        {
            ef->evaluate_residuals();
            ef->evaluate_chi2();
        }
        totalChi2 += ef->get_chi2();
    }
    return totalChi2;
}

void FGraphSolve::update_nodes()
{
    int acc_start = 0;
    for (uint_t i = 0; i < nodes_.size(); i++)
    {
        // node update is the negative of dx just calculated.
        // x = x - alpha * H^(-1) * Grad = x - dx
        // Depending on the optimization, it is already taking care of the step alpha, so we assume alpha = 1
        auto node_update = -dx_.block(acc_start, 0, nodes_[i]->get_dim(), 1);
        nodes_[i]->update(node_update);

        acc_start += nodes_[i]->get_dim();
    }
}

void FGraphSolve::synchronize_nodes_auxiliary_state()
{
    for (auto &&n : nodes_)
        n->set_auxiliary_state(n->get_state());
}


void FGraphSolve::synchronize_nodes_state()
{
    for (auto &&n : nodes_)
        n->set_state(n->get_auxiliary_state());
}

// method to output (to python) or other programs the current state of the system.
std::vector<MatX> FGraphSolve::get_estimated_state()
{
    vector<MatX> results;
    results.reserve(nodes_.size());

    for (uint_t i = 0; i < nodes_.size(); i++)
    {
        //nodes_[i]->print();
        MatX updated_pos = nodes_[i]->get_state();
        results.emplace_back(updated_pos);
    }

    return results;
}

MatX1 FGraphSolve::get_chi2_array()
{
    MatX1 results(factors_.size());

    for (uint_t i = 0; i < factors_.size(); ++i)
    {
        auto f = factors_[i];
        results(i) = f->get_chi2();
    }

    return results;
}
