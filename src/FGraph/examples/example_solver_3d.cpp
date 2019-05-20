/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * example_solver_3d.cpp
 *
 *  Created on: May 20, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */





#include "mrob/factor_graph_solve.hpp"
#include "mrob/factors/factor1Pose3d.hpp"
#include "mrob/factors/nodePose3d.hpp"


#include <iostream>

int main ()
{

    // create a simple graph to solve:
    //     anchor ------ X1 ------- obs ---------- X2
    mrob::FGraphSolve graph(mrob::FGraphSolve::CHOL_ADJ,50,50);

    // Initial node is defined at 0,0,0, 0,0,0 and anchor factor actually observing it at 0
    Mat61 x, obs;
    x = Mat61::Random()*0.1;
    obs = Mat61::Zero();
    std::shared_ptr<mrob::Node> n1(new mrob::NodePose3d(x));
    graph.add_node(n1);
    Mat6 obsInformation= Mat6::Identity();
    std::shared_ptr<mrob::Factor> f1(new mrob::Factor1Pose3d(obs,n1,obsInformation*1e6));
    graph.add_factor(f1);


    // solve the Gauss Newton optimization
    graph.print(true);
    graph.solve_batch();

    //TODO Something on the residuals and Jacobians is not rioght. The T is not what the node has stored!

    std::cout << "\nSolved, chi2 = " << graph.chi2() << std::endl;

    graph.print(true);
    return 0;
}
