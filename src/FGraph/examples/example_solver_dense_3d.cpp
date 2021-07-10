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
 * example_solver_3d.cpp
 *
 *  Created on: Nov 10, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */





#include "mrob/factor_graph_solve_dense.hpp"
#include "mrob/factors/nodePose3d.hpp"
#include "mrob/factors/factor1Pose3d.hpp"
#include "mrob/factors/factor2Poses3d.hpp"


#include <iostream>

int main ()
{

    // create a simple graph to solve:
    //     anchor ------ X1 ------- obs ---------- X2
    mrob::FGraphSolveDense graph;

    // Initial node is defined at 0,0,0, 0,0,0 and anchor factor actually observing it at 0
    mrob::Mat61 obs;
    mrob::Mat6 obsInformation= mrob::Mat6::Identity()*1e6;
    mrob::Mat61 x = mrob::Mat61::Random()*0.05;
    mrob::SE3 Tobs;
    mrob::SE3 Tx(x);
    std::shared_ptr<mrob::Node> n0(new mrob::NodePose3d(Tx));
    graph.add_node(n0);
    std::shared_ptr<mrob::Factor> f0(new mrob::Factor1Pose3d(Tobs,n0,obsInformation));
    graph.add_factor(f0);

    // Add a small chain
    std::shared_ptr<mrob::Node> n1(new mrob::NodePose3d(Tx));// it will be later updated by the factor
    graph.add_node(n1);
    obs << -0.1,0.2,0.5, 1,-2 ,3;
    Tobs = mrob::SE3(obs);
    obsInformation= mrob::Mat6::Identity();
    std::shared_ptr<mrob::Factor> f1(new mrob::Factor2Poses3d(Tobs,n0,n1,obsInformation, true));
    graph.add_factor(f1);

    // Add a small chain
    std::shared_ptr<mrob::Node> n2(new mrob::NodePose3d(Tx));// it will be later updated by the factor
    graph.add_node(n2);
    std::shared_ptr<mrob::Factor> f2(new mrob::Factor2Poses3d(Tobs,n1,n2,obsInformation, true));
    graph.add_factor(f2);

    // Closing the loop
    obs *= 2.0;
    std::shared_ptr<mrob::Factor> f3(new mrob::Factor2Poses3d(mrob::SE3(obs),n0,n2,obsInformation*1e2, true));
    graph.add_factor(f3);


    // solve the Gauss Newton optimization
    graph.print(true);
    graph.solve(mrob::Optimizer::NEWTON_RAPHSON);

    graph.print(true);
    std::cout << "\n\n\nSolved, chi2 = " << graph.calculate_error() << std::endl;
    return 0;
}



