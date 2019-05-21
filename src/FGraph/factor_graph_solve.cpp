/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
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

#include <chrono>
typedef std::chrono::microseconds Ttim;

using namespace mrob;
using namespace std;
using namespace Eigen;


FGraphSolve::FGraphSolve(solveMethod method, uint_t potNumberNodes, uint_t potNumberFactors):
	FGraph(potNumberNodes, potNumberFactors), method_(method)
{

}

FGraphSolve::~FGraphSolve() = default;


void FGraphSolve::solve_batch()
{
    /**
     * 2800 nodes
     * Time profile :13.902 % build Adjacency matrix,
     *               34.344 % build Information,
     *               48.0506 % build Cholesky,
     *               2.3075 % solve forward and back subtitution,
     *               1.3959 % update values,
     *
     */

    time_profiles_.clear();
    auto t1 = std::chrono::steady_clock::now();


    // 1) Linearizes and calculates the Jacobians and required matrices
    switch(method_)
    {
      case CHOL_ADJ:
        this->build_adjacency();
        this->build_info_adjacency();
        break;
      case CHOL:
        this->build_info_direct();
        break;
      case SCHUR:
      default:
        assert(0 && "FGraphSolve: method not implemented");
    }
    auto t2 = std::chrono::steady_clock::now();
    auto dif = std::chrono::duration_cast<Ttim>(t2 - t1);
    time_profiles_.push_back(dif.count());

    // 2) Cholesky factorization TODO SCHUR not implemented
    t1 = std::chrono::steady_clock::now();
    this->solve_cholesky();
    t2 = std::chrono::steady_clock::now();
    dif = std::chrono::duration_cast<Ttim>(t2 - t1);
    time_profiles_.push_back(dif.count());

    // 3) Update solution (this is almost negligible on time)
    t1 = std::chrono::steady_clock::now();
    this->update_nodes();
    t2 = std::chrono::steady_clock::now();
    dif = std::chrono::duration_cast<Ttim>(t2 - t1);
    time_profiles_.push_back(dif.count());

    double sum = 0;
    for (auto t : time_profiles_)
        sum += t;

    std::cout << "\nTime profile :";
    for (auto t : time_profiles_)
        std::cout << t/sum *100 << ", ";
    std::cout << "\n";
}

void FGraphSolve::solve_incremental()
{
    assert(0 && "FGraphSolve::solve_batch: Programm me");
}

void FGraphSolve::build_adjacency()
{
    // 0) resize properly matrices (if needed)
    r_.resize(obsDim_,1);//dense vector
    A_.resize(obsDim_, stateDim_);//Sparse matrix clear data
    W_.resize(obsDim_, obsDim_);//TODO should we reinitialize this all the time? an incremental should be fairly easy

    // 1) create the vector's structures
    std::vector<std::shared_ptr<Factor> >* factors;
    std::vector<std::shared_ptr<Node> >* nodes;
    // TODO: optimizing subgraph is not an option now, but we maintain generality
    factors = &factors_;
    nodes = &nodes_;

    // 2) vector structure to bookkeep the starting Nodes indices inside A

    // 2.2) Node indexes bookeept
    std::vector<uint_t> indNodesMatrix;
    indNodesMatrix.reserve(nodes->size());

    uint_t N = 0;
    for (id_t i = 0; i < nodes->size(); ++i)
    {
        // calculate the indices to access
        uint_t dim = (*nodes)[i]->get_dim();
        indNodesMatrix.push_back(N);
        N += dim;

    }
    assert(N == stateDim_ && "FGraphSolve::buildAdjacency: State Dimensions are not coincident\n");

    // 3) Evaluate every factor given the current state and bookeeping of Factor indices
    std::vector<uint_t> reservationA;
    reservationA.reserve( obsDim_ );
    std::vector<uint_t> reservationW;
    reservationW.reserve( obsDim_ );
    std::vector<uint_t> indFactorsMatrix;
    indFactorsMatrix.reserve(factors->size());
    uint_t M = 0;
    for (uint_t i = 0; i < factors->size(); ++i)
    {
        auto f = (*factors)[i];
        f->evaluate_residuals();
        f->evaluate_jacobians();

        // calculate dimensions for reservation and bookeping vector
        uint_t dim = f->get_dim();
        uint_t allDim = f->get_all_nodes_dim();
        for (uint_t j = 0; j < dim; ++j)
        {
            reservationA.push_back(allDim);
            reservationW.push_back(dim-j);
        }
        indFactorsMatrix.push_back(M);
        M += dim;
    }
    assert(M == obsDim_ && "FGraphSolve::buildAdjacency: Observation dimensions are not coincident\n");
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
                    uint_t iCol = indNodesMatrix[indNode] + k;
                    // XXX This is not an ordered insertion. Is it better to order and then insert?
                    A_.insert(iRow,iCol) = f->get_jacobian()(l, k + totalK);
                }
                totalK += dimNode;
            }
        }


        // 5) Get information matrix for every factor
        for (uint_t l = 0; l < f->get_dim(); ++l)
        {
            // only iterates over the upper triangular part
            for (uint_t k = l; k < f->get_dim(); ++k)
            {
                uint_t iRow = indFactorsMatrix[i] + l;
                uint_t iCol = indFactorsMatrix[i] + k;
                W_.insert(iRow,iCol) = f->get_information_matrix()(l,k);
                // If QR, then we needed, we dont suppoort QR anyway
                //W_.insert(iRow,iCol) = f->get_trans_sqrt_information_matrix()(l,k);
            }
        }
    } //end factors loop


}

void FGraphSolve::build_info_adjacency()
{
    /**
     * I_ dx = b_ corresponds to the normal equation A'*W*A dx = A'*W*r
     * only store the lower part of the information matrix (symmetric)
     *
     * XXX: In terms of speed, using the selfadjointview does not improve,
     * Eigen stores a temporary object and then copy only the upper part.
     *
     */
    I_ = (A_.transpose() * W_.selfadjointView<Eigen::Upper>() * A_);
    b_ = A_.transpose() * W_.selfadjointView<Eigen::Upper>() * r_;
}

matData_t FGraphSolve::chi2(bool evaluateResidualsFlag)
{
    matData_t totalChi2 = 0.0;
    for (uint_t i = 0; i < factors_.size(); ++i)
    {
        auto f = factors_[i];
        if (evaluateResidualsFlag)
            f->evaluate_residuals();
        f->evaluate_chi2();
        totalChi2 += f->get_chi2();
    }
    return totalChi2;
}


void FGraphSolve::build_info_direct()
{
    assert(0 && "FGraphSolve::buildProblemDirectInfo: Routine not implemented");
}

void FGraphSolve::solve_cholesky()
{
    // COLAMD is not supported on sparseCholesky
    SimplicialLDLT<SMatCol,Lower, AMDOrdering<SMatCol::StorageIndex>> cholesky;

    cholesky.compute(I_);
    dx_ = cholesky.solve(b_);

}

void FGraphSolve::update_nodes()
{
    int acc_start = 0;
    for (uint_t i = 0; i < nodes_.size(); i++)
    {
        // node update is the negative of dx just calculated.
        //x = x - alhpa * H^(-1) * Grad = x - dx    \ alpha = 1 since we are close to the solution
        auto node_update = -dx_.block(acc_start, 0, nodes_[i]->get_dim(), 1);
        nodes_[i]->update(node_update);

        acc_start += nodes_[i]->get_dim();
    }
}

// method to output (to python) or other programs the current state of the system.
std::vector<MatX1> FGraphSolve::get_estimated_state()
{
    vector<MatX1> results;
    results.reserve(nodes_.size());

    for (uint_t i = 0; i < nodes_.size(); i++) {
        MatX1 updated_pos = nodes_[i]->get_state();
        results.emplace_back(updated_pos);
    }

    return results;
}
