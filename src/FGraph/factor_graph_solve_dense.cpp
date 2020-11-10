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
 * factor_graph_solve_dense.cpp
 *
 *  Created on: Sep 22, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */


#include "mrob/factor_graph_solve_dense.hpp"
#include <iostream>

using namespace mrob;

FGraphSolveDense::FGraphSolveDense():
        FGraph(), Optimizer()
{

}

FGraphSolveDense::~FGraphSolveDense()
{

}


// Function from the parent class Optimizer.
// This calculate the total chi2 of the graph, and re-evaluates residuals
matData_t FGraphSolveDense::calculate_error()
{
    matData_t totalChi2 = 0.0;
    for (uint_t i = 0; i < factors_.size(); ++i)
    {
        auto f = factors_[i];
        f->evaluate_residuals();
        f->evaluate_chi2();
        totalChi2 += f->get_chi2();
    }
    return totalChi2;
}
void FGraphSolveDense::calculate_gradient_hessian()
{
    // 1) calculate dimensionality and resize properly
    uint_t n = this->get_dimension_state();
    gradient_.resize(n);
    gradient_.setZero();
    hessian_.resize(n,n);
    hessian_.setZero();

    // 2) evaluate residuals and Jacobians
    for (uint_t i = 0; i < factors_.size(); ++i)
    {
        auto f = factors_[i];
        // XXX: If using some flags, this should no be needed (only for a case in LM)
        f->evaluate_residuals();
        f->evaluate_jacobians();
        f->evaluate_chi2();
    }

    // 3) calculate the indexes for the nodes for block allocation
    std::vector<uint_t> indNodesMatrix;
    indNodesMatrix.reserve(nodes_.size());
    uint_t N = 0, M = 0;
    for (id_t i = 0; i < nodes_.size(); ++i)
    {
        // calculate the indices to access
        uint_t dim = nodes_[i]->get_dim();
        indNodesMatrix.push_back(N);
        N += dim;
    }
    assert(N == stateDim_ && "FGraphSolveDense::HessianAndGradient: State Dimensions are not coincident\n");

    // 4) create Hessian and gradient, by traversing all nodes, looking for its factors
    //    and completing the rows in the Hessian and gradient.
    for (uint_t n = 0; n < nodes_.size(); ++n)
    {
        auto node = nodes_[n];
        uint_t node_id =  node->get_id();
        N = node->get_dim();
        auto factors = node->get_neighbour_factors();
        for ( uint_t i = 0; i < factors->size(); ++i )
        {
            auto f = factors_[i];
            auto nodes_connected = f->get_neighbour_nodes();
            MatX J = f->get_jacobian();
            uint_t D = f->get_dim();

            // get the trans Jacobian corresponding to the node n. Always premultiplies in Hessian
            MatX J_n_t;
            uint_t matrix_index = 0;
            for (uint_t m = 0; m < nodes_connected->size() ; ++m)
            {
                std::cout << "node " << m << std::endl;
                if ((*nodes_connected)[m]->get_id() ==  node_id)
                {
                    // TODO sometime it misses this point, why? fix this
                    J_n_t = J.block(0, matrix_index, D,N ).transpose();
                    continue;
                }
                matrix_index += (*nodes_connected)[m]->get_dim();
            }

            // Calculate the second part corresponding on the second factor
            matrix_index = 0;
            for (uint_t m = 0; m < nodes_connected->size() ; ++m)
            {
                auto node2 = (*nodes_connected)[m];
                M = node2->get_dim();
                MatX J_m = J.block(0, matrix_index, D,M);
                matrix_index += M;
                // Gradient: Grad(n) = \sum J_n_t*W*r
                gradient_.segment(indNodesMatrix[n], N) +=
                        J_n_t * f->get_information_matrix() * f->get_residual();
                // Hessian: H(n,m) = \sum J_n_t'*W*J_m
                // Only the current block row is filled in (corresponding to the n-node)
                hessian_.block(indNodesMatrix[n], indNodesMatrix[m],N,M) +=
                        J_n_t * f->get_information_matrix() * J_m;
            }
        }
    }
    std::cout << "hessian matrix> \n" << gradient_ << std::endl;

}
void FGraphSolveDense::update_state(const MatX1 &dx)
{
    int acc_start = 0;
    for (uint_t i = 0; i < nodes_.size(); i++)
    {
        // node update is the negative of dx just calculated.
        // x = x - alpha * H^(-1) * Grad = x - dx
        // Depending on the optimization, it is already taking care of the step alpha, so we assume alpha = 1
        auto node_update = dx_.block(acc_start, 0, nodes_[i]->get_dim(), 1);
        nodes_[i]->update(node_update);

        acc_start += nodes_[i]->get_dim();
    }
}
void FGraphSolveDense::bookkeep_state()
{
    for (auto n : nodes_)
        n->set_auxiliary_state(n->get_state());
}
void FGraphSolveDense::update_state_from_bookkeep()
{
    for (auto n : nodes_)
        n->set_state(n->get_auxiliary_state());
}

