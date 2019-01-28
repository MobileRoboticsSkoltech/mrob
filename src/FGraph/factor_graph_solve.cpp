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
#include "mrob/CustomCholesky.h"

#include <iostream>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>
#include <Eigen/SparseCholesky>
#include <mrob/factor_graph_solve.hpp>


using namespace mrob;
using namespace std;
using namespace Eigen;


FGraphSolve::FGraphSolve(solveType type, uint_t potNumberNodes, uint_t potNumberFactors):
	FGraph(potNumberNodes, potNumberFactors), type_(type),
	last_solved_node(-1), last_solved_factor(-1)
{

}

FGraphSolve::~FGraphSolve() = default;


void FGraphSolve::buildProblem()
{
    switch(type_)
    {
    case QR:
    case CHOL_ADJ:
        buildAdjacency();
        break;
    case CHOL:
    case SCHUR:
    default:
        assert(0 && "FGraphSolve::buildProblem: Method not implemented\n");
        break;
    }
}

void FGraphSolve::solveOnce()
{
    solveChol();

    // Keep indices of last node and factor
    last_solved_node = nodes_.size() - 1;
    last_solved_factor = factors_.size() - 1;
}

void FGraphSolve::solveIncremental()
{
    // TODO incremental update should check for loop closures
    // NOTE: it works only with odometry factors
    solveCholIncremental();

    // Keep indices of last node and factor
    last_solved_node = nodes_.size() - 1;
    last_solved_factor = factors_.size() - 1;
}


void FGraphSolve::buildAdjacency()
{
    // 0) resize properly matrices (if needed)
    r_.resize(obsDim_,1);//dense vector
    A_.resize(obsDim_, stateDim_);//Sparse matrix clear data
    W_.resize(obsDim_, obsDim_);//XXX shoulld we reinitialize this all the time? an incremental should be fairly easy

    // 1) create the vector's structures to iterate faster
    std::vector<std::shared_ptr<Factor> >* factors;
    std::vector<std::shared_ptr<Node> >* nodes;
    if (isHoleProblem_)
    {
        factors = &factors_;
        nodes = &nodes_;
    }
    else
    {
        // XXX this option is not working now
        factors = &localFactors_;
        nodes = &localNodes_;
    }

    // 2) vector structure to bookkeep the starting Nodes indices inside A
    std::vector<uint_t> indNodesMatrix;
    indNodesMatrix.reserve(nodes->size());
    uint_t N = 0;
    for (uint_t i = 0; i < nodes->size(); ++i)
    {
        // calculate the indices to access
        uint_t dim = (*nodes)[i]->getDim();
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
        f->evaluate();

        // calculate dimensions for reservation and bookeping vector
        uint_t dim = f->getDim();
        uint_t allDim = f->getAllNodesDim();
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


    // This could be subject to parallelization, maybe on two steps: eval + build
    for (uint_t i = 0; i < factors->size(); ++i)
    {
        auto f = (*factors)[i];

        // 4) Get the calculated residual
        r_.block(indFactorsMatrix[i], 0, f->getDim(), 1) << f->getResidual();

        // 5) build Adjacency matrix as a composition of rows
        // 5.1) Get the number of nodes involved. It is a sorted list
        auto neighNodes = f->getNeighbourNodes();
        //TODO test if a local variable for jacobian speepds up things
        // Iterates over the Jacobian row
        for (uint_t l=0; l < f->getDim() ; ++l)
        {
            uint_t totalK = 0;
            // Iterates over the number of neighbour Nodes (ordered by construction)
            for (uint_t j=0; j < neighNodes->size(); ++j)
            {
                uint_t indNode = (*neighNodes)[j]->getId() -1;//Ids start at 1, while vector at 0
                uint_t dimNode = (*neighNodes)[j]->getDim();
                for(uint_t k = 0; k < dimNode; ++k)
                {
                    uint_t iRow = indFactorsMatrix[i] + l;
                    uint_t iCol = indNodesMatrix[indNode] + k;
                    // This is an ordered insertion
                    A_.insert(iRow,iCol) = f->getJacobian()(l, k + totalK);
                }
                totalK += dimNode;
            }
        }


        // 5) Get information matrix for every factor, ONLY for the QR we need W^T/2
        for (uint_t l =0; l < f->getDim(); ++l)
        {
            // only iterates over the upper triangular part
            for (uint_t k = l; k < f->getDim(); ++k)
            {
                uint_t iRow = indFactorsMatrix[i] + l;
                uint_t iCol = indFactorsMatrix[i] + k;
                if (QR)
                {
                    W_.insert(iRow,iCol) = f->getWT2()(l,k);
                }
                else
                {
                    W_.insert(iRow,iCol) = f->getInvCovariance()(l,k);
                }
            }
        }
    } //end factors loop

//    std::cout << A_ << std::endl;
//    std::cout << W_ << std::endl;
//    cout << endl << r_ << endl;
}

void FGraphSolve::buildAdjacency(SMat &A_new, SMat &W_new, MatX1 &r_new) {
    auto    state_dim = stateDim_ - I_.cols() + (last_solved_node == -1 ? 0 : nodes_[last_solved_node]->getDim()) ,
            obs_dim = obsDim_ - I_.rows();
    A_new.resize(obs_dim, state_dim);
    W_new.resize(obs_dim, obs_dim);
    r_new.resize(obs_dim, 1);

    std::vector<uint_t> reservationA, reservationW;

    reservationA.reserve(static_cast<unsigned long>(obs_dim));
    reservationW.reserve(static_cast<unsigned long>(obs_dim));

    auto last_factors_count = last_solved_factor + 1;
    auto states = nodes_.size() - last_solved_node, dims = factors_.size() - last_factors_count;

    std::vector<uint_t> indNodesMatrix;
    uint_t N = 0;

    indNodesMatrix.reserve(states);

    for(auto i = nodes_.begin() + states; i != nodes_.end(); ++i) {
        uint_t dim = (*i)->getDim();
        indNodesMatrix.push_back(N);
        N += dim;
    }

    std::vector<uint_t> indFactorsMatrix;
    uint_t M = 0;

    indFactorsMatrix.reserve(dims);

    for(auto i = factors_.begin() + states; i != factors_.end(); ++i) {
        (*i)->evaluate();

        uint_t dim = (*i)->getDim();
        uint_t allDim = (*i)->getAllNodesDim();
        for (uint_t j = 0; j < dim; ++j)
        {
            reservationA.push_back(allDim);
            reservationW.push_back(dim - j);
        }
        indFactorsMatrix.push_back(M);
        M += dim;
    }

    A_new.reserve(reservationA);
    W_new.reserve(reservationW);

    for (auto i = last_factors_count; i < factors_.size(); i++) {
        auto f = factors_[i];

        r_new.block(indFactorsMatrix[i - last_factors_count], 0, f->getDim(), 1) << f->getResidual();

        auto neighNodes = f->getNeighbourNodes();
        for (uint_t l = 0; l < f->getDim() ; ++l) {
            uint_t totalK = 0;
            for (auto &j: *neighNodes) {
                uint_t indNode = j->getId() - 1 - last_solved_node;
                uint_t dimNode = j->getDim();
                for (uint_t k = 0; k < dimNode; ++k) {
                    uint_t iRow = indFactorsMatrix[i - last_factors_count] + l;
                    uint_t iCol = indNodesMatrix[indNode] + k;

                    A_new.insert(iRow, iCol) = f->getJacobian()(l, k + totalK);
                }
                totalK += dimNode;
            }
        }


        for (uint_t l = 0; l < f->getDim(); ++l) {
            for (uint_t k = l; k < f->getDim(); ++k) {
                uint_t iRow = indFactorsMatrix[i - last_factors_count] + l;
                uint_t iCol = indFactorsMatrix[i - last_factors_count] + k;
                W_new.insert(iRow, iCol) = f->getInvCovariance()(l, k);
            }
        }
    }

//    cout << A_new.toDense() << endl;
//    cout << W_new.toDense() << endl;
//    cout << r_new << endl;
}


void FGraphSolve::buildDirectInfo()
{
    assert(0 && "FGraphSolve::buildProblemDirectInfo: Routine not implemented");
}

void FGraphSolve::solveQR()
{
    // W^T/2 corresponds to the upper triangular matrix, so we keep that format
    A_ = W_.triangularView<Eigen::Upper>() * A_;//creates a compressed Matrix
    b_ = W_.triangularView<Eigen::Upper>() * r_;
}

void FGraphSolve::solveChol()
{
    /**
     * I_ dx = b_ corresponds to the normal equation A'*W*A dx = A'*W*r
     * only store the lower part of the information matrix (symmetric)
     */
    I_ = (A_.transpose() * W_.selfadjointView<Eigen::Upper>() * A_).selfadjointView<Eigen::Upper>();
    b_ = A_.transpose() * W_.selfadjointView<Eigen::Upper>() * r_;

//    cout << I_.toDense() << endl;
//    std::cout << b_ << std::endl;

    /**
     * Solve LSQ via sparse Cholesky
     * TODO reordering
     */
    CustomCholesky<SMat> cholesky(I_);
    L_ = cholesky.getL();
    auto dx_ = cholesky.solve(b_);

    cout << L_.toDense() << endl;
//    cout << dx_ << endl;

//    updateNodes(dx_);
}


void FGraphSolve::solveCholIncremental() {
    // 0) Build information form of new update
    SMat A_new, W_new;
    MatX1 r_new;
    buildAdjacency(A_new, W_new, r_new);

    SMat I_new = (A_new.transpose() * W_new.selfadjointView<Eigen::Upper>() * A_new).selfadjointView<Eigen::Upper>();
    MatX1 b_new = A_new.transpose() * W_new.selfadjointView<Eigen::Upper>() * r_new;

//    cout << I_new.toDense()<< endl;
//    cout << b_new << endl;

//    incrementalViaInformation(I_new, b_new);
    incrementalViaL(I_new, b_new);
}

void FGraphSolve::incrementalViaInformation(SMat &I_new, MatX1 &b_new) {
    // 1) Allocate memory for update
    SMat I_updated;
    MatX1 b_updated;

    I_updated.resize(stateDim_, stateDim_);
    b_updated.resize(stateDim_, 1);

    std::vector<uint_t> reservationI;
    reservationI.reserve(stateDim_);

    // Put I_ space usage first
    for (int i = 0; i < I_.outerSize(); i++) {
        auto    curr = *(I_.outerIndexPtr() + i),
                next = *(I_.outerIndexPtr() + i + 1);
        reservationI.push_back(static_cast<unsigned int &&>(next - curr));
    }

    // Then add I_new terms
    int intersection = (last_solved_node == -1 ? 0 : nodes_[last_solved_node]->getDim()),
            starting_index = static_cast<int>(I_.cols()) - intersection;

    for(int i = 0; i < intersection; i++) {
        auto    curr = *(I_new.outerIndexPtr() + i),
                next = *(I_new.outerIndexPtr() + i + 1);
        reservationI[starting_index + i] += static_cast<unsigned int>(next - curr) - intersection;
    }

    for (int i = intersection; i < I_new.outerSize(); i++) {
        auto    curr = *(I_new.outerIndexPtr() + i),
                next = *(I_new.outerIndexPtr() + i + 1);
        reservationI.push_back(static_cast<unsigned int>(next - curr));
    }

    I_updated.reserve(reservationI);

    // 2) Join I_ and I_new together
    vector<Eigen::Triplet<matData_t>> tripletList;

    for (int i = 0; i < I_.outerSize(); ++i) {
        for (SMat::InnerIterator it(I_, i); it; ++it) {
            tripletList.emplace_back(it.row(), it.col(), it.value());
        }
    }

    for (int i = 0; i < I_new.outerSize(); ++i) {
        for (SMat::InnerIterator it(I_new, i); it; ++it) {
            tripletList.emplace_back(it.row() + starting_index, it.col() + starting_index, it.value());
        }
    }

    I_updated.setFromTriplets(tripletList.begin(), tripletList.end());

    // 3) Join b_ and b_new together
    b_updated.block(0, 0, b_.rows(), 1) << b_;
    b_updated.block(starting_index, 0, b_new.rows(), 1) += b_new;

//    cout << I_updated.toDense() << endl;
//    cout << b_updated << endl;

    // 4) Solve it
    SimplicialLLT<SMat> chol(I_updated);
    MatX1 dx_ = chol.solve(b_updated);

    //TODO we probalbliy should save updated values

    cout << dx_ << endl;

//    updateNodes(dx_);
}

void FGraphSolve::incrementalViaL(SMat &I_new, MatX1 &b_new) {

}


void FGraphSolve::updateNodes(const MatX1 &dx_) {
    int acc_start = 0;
    for (auto &node : nodes_) {
        auto node_update = dx_.block(acc_start, 0, node->getDim(), 1);
        node->update(node_update);

        acc_start += node->getDim();
    }
}
