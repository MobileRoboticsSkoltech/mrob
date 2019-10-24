/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * example_solver_3d.cpp
 *
 *  Created on: May 20, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */





#include "mrob/factor_graph_solve.hpp"
#include "mrob/factors/nodePose3d.hpp"
#include "mrob/factors/factor1Pose3d.hpp"
#include "mrob/factors/factor2Poses3d.hpp"


#include <iostream>

int main ()
{

    // create a simple graph to solve:
    //     anchor ------ X1 ------- obs ---------- X2
    mrob::FGraphSolve graph(mrob::FGraphSolve::ADJ,mrob::FGraphSolve::GN);

    // Initial node is defined at 0,0,0, 0,0,0 and anchor factor actually observing it at 0
    Mat61 x, obs;
    Mat6 obsInformation= Mat6::Identity();
    x = Mat61::Random()*0.05;
    obs = Mat61::Zero();
    std::shared_ptr<mrob::Node> n0(new mrob::NodePose3d(x));
    graph.add_node(n0);
    std::shared_ptr<mrob::Factor> f0(new mrob::Factor1Pose3d(obs,n0,obsInformation*1e6));
    graph.add_factor(f0);

    // Add a small chain
    std::shared_ptr<mrob::Node> n1(new mrob::NodePose3d(x));// it will be later updated by the factor
    graph.add_node(n1);
    obs << -0.1,0.2,0.5, 1,-2 ,3;
    std::shared_ptr<mrob::Factor> f1(new mrob::Factor2Poses3d(obs,n0,n1,obsInformation, true));
    graph.add_factor(f1);

    // Add a small chain
    std::shared_ptr<mrob::Node> n2(new mrob::NodePose3d(x));// it will be later updated by the factor
    graph.add_node(n2);
    std::shared_ptr<mrob::Factor> f2(new mrob::Factor2Poses3d(obs,n1,n2,obsInformation, true));
    graph.add_factor(f2);

    // Closing the loop
    obs *= 2.0;
    std::shared_ptr<mrob::Factor> f3(new mrob::Factor2Poses3d(obs,n0,n2,1e2*obsInformation, true));
    graph.add_factor(f3);


    // solve the Gauss Newton optimization
    graph.print(true);
    graph.solve();
    graph.solve();
    graph.solve();
    graph.solve();

    graph.print(true);
    std::cout << "\n\n\nSolved, chi2 = " << graph.chi2() << std::endl;
    return 0;
}
