/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * DenseGaussNewton.cpp
 *
 *  Created on: Mar 3, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "solverDense.hpp"
#include <iostream>

using namespace fg;

DenseGaussNewton::DenseGaussNewton(std::shared_ptr<FGraph> fg):
        fg_(fg), isProblemEvaluated(false)
{
    fg->print();
    //lowerTriangularInformation_.selfadjointView<Eigen::Lower>();
    // Create the matrices on Ax=b
    uint_t state_dim = 3;//read from fg?
    MatX Information_(state_dim, state_dim);
    MatX1 b_(state_dim);
}

DenseGaussNewton::~DenseGaussNewton()
{

}

void DenseGaussNewton::solveOnce()
{
	// 1) Evaluate, this updates Information, r and b
	if (isProblemEvaluated)
		this->evaluate();

	// 2) Factorize Information to solve I dx = b

	// 3) Update solution x := x + dx

    // After solving the problem, we would need a new evaluation
	isProblemEvaluated = false;
}

int DenseGaussNewton::solve()
{
    // Iteratively solve until convergence
    return 0;
}

/*MatX Factor2Poses3d::getJacobian(std::shared_ptr<Node> &n) const
{
    if(neighbourNodes_[0] == n)
        return J1_;
    if(neighbourNodes_[1] == n)
        return J2_;
    else
        // derivatives w.r.t other nodes are 0, regardless of the incorrectness of trying
        // get a Jacobian that does not define the factor
        return Mat6::Zero();
}*/

void DenseGaussNewton::evaluate()
{
	// 1) Iterate over all factors
	auto iter = fg_->getBeginFactorsIter();
	for ( ; iter != fg_->getEndFactorsIter(); ++iter )
	{
		// 2) Every Factor evaluates its error and accumulates the Chi2
		std::shared_ptr<fg::Factor> factor = (*iter);
		//internally updates r and J_
		factor->evaluate();
	}

}
matData_t DenseGaussNewton::evaluateChi2()
{
	// 1) Iterate over all factors
	auto iter = fg_->getBeginFactorsIter();
	matData_t chi2 = 0.0;
	for ( ; iter != fg_->getEndFactorsIter(); ++iter )
	{
		// 2) Every Factor evaluates its error and accumulates the Chi2
		std::shared_ptr<fg::Factor> factor = (*iter);
		//internally updates r
		factor->evaluateError();

		// chi2_f = r'*W*r
		chi2 += factor->getChi2();
	}
	return chi2;
}

void DenseGaussNewton::buildMatrices()
{

	// 1) Iterate over all nodes, for now, we don't elect a subset of them TODO
	auto iter = fg_->getBeginNodesIter();
	for ( ; iter != fg_->getEndNodesIter(); ++iter )
	{
		// 2) Every Node has a vector of neighbour factors, we iterate over them to calculate
		// residuals and the Jacobians
		uint_t nodeD = (*iter)->getDim();
		auto neighFactors = (*iter)->getNeighbourFactors();
		for (uint_t i = 0; i < neighFactors->size();  ++i)
		{
			std::shared_ptr<fg::Factor> factor = (*neighFactors)[i];//shared pointer to a base class factor
			uint_t factD = factor->getDim();
			// Get residual
			MatD1<factD> r;
			factor->getResidual(r);


			// Get Jacobian, with respect to all nodes
			uint_t allNodesDim = factor->getAllNodesDim();
			factor->getJacobian(J);
			MatD<factD,nodeD> Jn = J.block<>;
		}

	}

	// flag to control if evaluation has been done or not.
	isProblemEvaluated = true;
}
