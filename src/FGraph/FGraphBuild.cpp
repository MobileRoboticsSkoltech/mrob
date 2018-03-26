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
    case Adjacency:
        buildProblemAdjacency();
        break;
    case Info:
        buildProblemDirectInfo();
        break;
    case Adj2Info:
    default:
        buildProblemAdj2Info();
        break;
    }
}


void FGraphBuild::buildProblemAdjacency()
{

}


void FGraphBuild::buildProblemAdj2Info()
{

}

void FGraphBuild::buildProblemDirectInfo()
{

}
