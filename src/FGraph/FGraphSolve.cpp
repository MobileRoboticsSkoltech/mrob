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
#include "FGraphSolve.hpp"

using namespace fg;

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
        assert(0 && "FGraphSolve::buildProblem: Methods not implemented\n");
        break;
    }
}

void FGraphSolve::solveQR()
{
    //buildAdjacency(); //has been called previously
    A_ = W_.selfadjointView<Eigen::Lower>() * A_;
    b_ = W_.selfadjointView<Eigen::Lower>() * r_;
}

void FGraphSolve::solveChol()
{
    // only store the lower part of the information matrix (symetric)
    I_ = (A_.transpose() * W_.selfadjointView<Eigen::Lower>() * A_).selfadjointView<Eigen::Lower>();
    b_ = A_.transpose() * W_.selfadjointView<Eigen::Lower>() * r_;
}

void FGraphSolve::buildAdjacency()
{
    // 0) resize properly matrices (if needed)
    r_.resize(obsDim_,1);//dense vector
    A_.resize(obsDim_, stateDim_);//Sparse matrix, should we clear it?
    A_.reserve((uint_t)2.5*obsDim_*6);//number of nonzeros. TODO read the average node dim

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
        factors = &localFactors_;
        nodes = &localNodes_;
    }

    for (uint_t i = 0; i < nodes->size(); ++i)
    {
        // calculate the indices to access
    }

    for (uint_t i = 0; i < factors->size(); ++i)
    {
        auto f = (*factors)[i];
        // 2) Evaluate every factor given the current state
        f->evaluate();

        // caulcualte dimesions for revervtaion
    }


    uint_t index = 0;
    // This could be subject to parallelization, maybe on two steps: eval + build
    for (uint_t i = 0; i < factors->size(); ++i)
    {
        auto f = (*factors)[i];

        // 3) Get the calculated residual and build the joint Residual
        r_.block(index, 0, f->getDim(), 1) << f->getResidual();

        // 4) build Adjacency matrix as a composition of rows
        // 4.1) Get the number of nodes involved
        auto neighNodes = f->getNeighbourNodes();
        for (uint_t i=0; i < neighNodes->size(); ++i)
        {
            //TODO
            (*neighNodes)[i]->getId();
        }

        // 5) Get information matrix for every factor, ONLY for the QR we need W^T/2
        MatX W;
        if (QR)
        {
            W = f->getWT2();
        }
        else
        {
            W = f->getInvCovariance();
        }


        // update new indixes
        index += f->getDim();
    }
}


void FGraphSolve::buildDirectInfo()
{
    assert(0 && "FGraphSolve::buildProblemDirectInfo: Routine not implemented");
}
