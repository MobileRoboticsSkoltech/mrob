/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * FGraphSolve.cpp
 *
 *  Created on: Mar 23, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include <iostream>
#include "skmr/FGraphSolve.hpp"

using namespace skmr;

FGraphSolve::FGraphSolve(solveType type, uint_t potNumberNodes, uint_t potNumberFactors):
	FGraph(potNumberNodes, potNumberFactors), type_(type)
{

}

FGraphSolve::~FGraphSolve()
{

}

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

}

void FGraphSolve::solveQR()
{
    // W^T/2 corresponds to the upper triangular matrix, so we keep that format
    A_ = W_.triangularView<Eigen::Upper>() * A_;//creates a compressed Matrix
    b_ = W_.triangularView<Eigen::Upper>() * r_;
}

void FGraphSolve::solveChol()
{
    // I_ dx = b_ corresponds to the normal equation A'*W*A dx = A'*W*r
    // only store the lower part of the information matrix (symetric)
    I_ = (A_.transpose() * W_.selfadjointView<Eigen::Upper>() * A_).selfadjointView<Eigen::Upper>();
    b_ = A_.transpose() * W_.selfadjointView<Eigen::Upper>() * r_;
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
    A_.reserve(reservationA);//Exact allocation for elements.
    W_.reserve(reservationW);//same


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
            // Itereates over the number of neighbour Nodes (ordered by construction)
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
    }//end factors loop
    std::cout << W_ << std::endl;
}


void FGraphSolve::buildDirectInfo()
{
    assert(0 && "FGraphSolve::buildProblemDirectInfo: Routine not implemented");
}
