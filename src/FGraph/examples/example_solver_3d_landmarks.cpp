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
 * example_solver_3d_landmarks.cpp
 *
 *  Created on: March 21, 2020
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */





#include "mrob/factors/nodePose3d.hpp"
#include "mrob/factors/nodeLandmark3d.hpp"
#include "mrob/factors/factor1Pose3d.hpp"
#include "mrob/factors/factor1Pose1Landmark3d.hpp"
#include "mrob/factor_graph_solve.hpp"


#include <iostream>

int main ()
{

    // create a simple graph to solve:
    //    anchor ---- X1 ------- obs ---------- L1,2,3
    mrob::FGraphSolve graph(mrob::FGraphSolve::ADJ,mrob::FGraphSolve::GN);

    // Initial node is defined at 0,0,0, 0,0,0 and anchor factor actually observing it at 0
    Mat61 x, obs;
    x = Mat61::Random()*0.05;
    mrob::SE3 Tx(x);
    std::shared_ptr<mrob::Node> n0(new mrob::NodePose3d(Tx));
    graph.add_node(n0);
    Mat4 Tobs = Mat4::Identity();
    Mat6 obsInformation = Mat6::Identity();
    std::shared_ptr<mrob::Factor> f0(new mrob::Factor1Pose3d(Tobs,n0,obsInformation*1e6));
    graph.add_factor(f0);

    // Add Ladmarks: uninitialized
    Mat31 land;
    land << 0,0,0;
    std::shared_ptr<mrob::Node> l1(new mrob::NodeLandmark3d(land));// it will be later updated by the factor
    graph.add_node(l1);
    std::shared_ptr<mrob::Node> l2(new mrob::NodeLandmark3d(land));// it will be later updated by the factor
    graph.add_node(l2);
    std::shared_ptr<mrob::Node> l3(new mrob::NodeLandmark3d(land));// it will be later updated by the factor
    graph.add_node(l3);
    
    // reuse of variable, now this is the observation of the landmark
    Mat3 landInf = Mat3::Identity();
    land << 1,0,0;
    std::shared_ptr<mrob::Factor> f1(new mrob::Factor1Pose1Landmark3d(land,n0,l1,landInf));
    graph.add_factor(f1);
    
    land << 1,1,0;
    std::shared_ptr<mrob::Factor> f2(new mrob::Factor1Pose1Landmark3d(land,n0,l2,landInf));
    graph.add_factor(f2);
    
    land << 1,1,1;
    std::shared_ptr<mrob::Factor> f3(new mrob::Factor1Pose1Landmark3d(land,n0,l3,landInf));
    graph.add_factor(f3);

    // solve the Gauss Newton optimization
    graph.print(true);
    graph.solve(mrob::FGraphSolve::LM);

    std::cout << "\n\n\nSolved, chi2 = " << graph.chi2() << std::endl;
    graph.print(true);
    return 0;
}
