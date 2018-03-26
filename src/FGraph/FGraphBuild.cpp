/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * FGraphBuild.cpp
 *
 *  Created on: Mar 23, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include "FGraphBuild.hpp"

using namespace fg;

FGraphBuild::FGraphBuild(buildType type, uint_t potNumberNodes, uint_t potNumberFactors):
	FGraph(potNumberNodes, potNumberFactors), type_(type)
{

}

FGraphBuild::~FGraphBuild()
{

}

void FGraphBuild::buildProblem()
{
    switch(type_)
    {
    case ADJACENCY:
        buildProblemAdjacency();
        break;
    case INFO:
        buildProblemDirectInfo();
        break;
    case ADJ2INFO:
        buildProblemAdj2Info();
        break;
    case SCHUR:
    default:
        break;
    }
}


void FGraphBuild::buildProblemAdjacency()
{
    // 1) resize properly matrices (if needed)
    r_.resize(obsDim_,1);
    A_.resize(obsDim_, stateDim_);

    uint_t index = 0;
    for (uint_t i = 0; i < factors_.size(); ++i)
    {
        auto f = factors_[i];
        // 2) Evaluate every factor given the current state
        f->evaluate();

        // 3) Get the calculated residual and build the joint Residual
        r_.block(index, 0, f->getDim(), 1) << f->getResidual();

        // 4) Get information matrix for every factor
        const MatX* W = f->getInvCovariance();

        // 5) build Adjacency matrix as a composition of rows
        const MatX* J = f->getJacobian();

        // update new indixes
        index += f->getDim();
    }
}


void FGraphBuild::buildProblemAdj2Info()
{

}

void FGraphBuild::buildProblemDirectInfo()
{

}
