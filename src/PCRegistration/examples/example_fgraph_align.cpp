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
 * example_fgraph_align.cpp
 *
 *  Created on: Jan 15, 2021
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */



#include "mrob/factor_graph_solve.hpp"
#include "mrob/factors/nodePose3d.hpp"
#include "mrob/factors/factor1PosePoint2Plane.hpp"



#include <iostream>

int main ()
{
    mrob::FGraphSolve graph(mrob::FGraphSolve::ADJ,mrob::FGraphSolve::GN);

    std::shared_ptr<mrob::Node> n0(new mrob::NodePose3d(mrob::SE3()));
    graph.add_node(n0);


    // add observations, for now just random things
    Mat1 W;
    W << 1.0;

    for (uint_t t = 0; t < 10; t++)
    {
        Mat31 z_normal_y  = Mat31::Random();
        Mat31 z_point_y = Mat31::Random();
        Mat31 z_point_x = Mat31::Random();
        std::shared_ptr<mrob::Factor> f1(new mrob::Factor1PosePoint2Plane(z_point_x,z_point_y, z_normal_y,n0,W));
        graph.add_factor(f1);
    }
    graph.print();

    // solve the graph
    double chi2 = graph.chi2();
    std::cout << "Current error = " << chi2;

    // Evaluate the graph
    graph.solve();

    // solve the graph
    std::cout << "Current error = " <<  graph.chi2();
}
