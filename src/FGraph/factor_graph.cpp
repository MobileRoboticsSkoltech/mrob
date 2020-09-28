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
 * factor_graph.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include <iostream>
#include <mrob/factor_graph.hpp>


using namespace mrob;


FGraph::FGraph() :
        stateDim_(0),obsDim_(0)
{
}
FGraph::~FGraph()
{
    // clear every node's list of neigbours factors
    for (auto n: nodes_)
        n->clear();
    factors_.clear();
    nodes_.clear();
}

bool FGraph::add_factor(std::shared_ptr<Factor> &factor)
{
	factor->set_id(factors_.size());
	factors_.push_back(factor);
    auto list = factor->get_neighbour_nodes();
    for( auto n: *list)
    {
        n->add_factor(factor);
    }
    obsDim_ += factor->get_dim();
    return true;
}

bool FGraph::add_node(std::shared_ptr<Node> &node)
{
	node->set_id(nodes_.size());
	nodes_.push_back(node);
	stateDim_ += node->get_dim();
	return true;
}

std::shared_ptr<Node>& FGraph::get_node(uint_t key)
{
    // TODO key on a set or map?
    assert(key < nodes_.size() && "FGraph::get_node: incorrect key");
    return nodes_[key];// XXX test key  again
}

std::shared_ptr<Factor>& FGraph::get_factor(uint_t key)
{
    // TODO key on a set or map?
    assert(key < factors_.size() && "FGraph::get_node: incorrect key");
    return factors_[key];
}

matData_t FGraph::get_factor_chi2(uint_t key)
{
    auto f = this->get_factor(key);
    return f->get_chi2();
}

matData_t FGraph::evaluate_factor_chi2(uint_t key)
{
    auto f = this->get_factor(key);
    f->evaluate_residuals();
    f->evaluate_chi2();
    return f->get_chi2();
}

void FGraph::print(bool completePrint) const
{
    std::cout << "Status of graph: " <<
            nodes_.size()  << "Nodes and " <<
            factors_.size() << "Factors." << std::endl;

    if(completePrint)
    {
        for (auto n : nodes_)
            n->print();
        for (auto f : factors_)
            f->print();
    }
}
