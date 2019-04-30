/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * example_solver.cpp
 *
 *  Created on: April 10, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */





#include "mrob/factor_graph_solve.hpp"
#include "mrob/factors/factor1Pose2d.hpp"
#include "mrob/factors/factor2Poses2d.hpp"
#include "mrob/factors/nodePose2d.hpp"


#include <iostream>

int main ()
{

    // create a simple graph to solve:
    //     anchor ------ X1 ------- obs ---------- X2
    mrob::FGraphSolve graph(mrob::FGraphSolve::CHOL_ADJ,50,50);

    // Initial node is defined at 0,0,0, and anchor factor actually observing it at 0
    Mat31 x, obs;
    x = Mat31::Zero();
    obs = Mat31::Zero();
    std::shared_ptr<mrob::Node> n1(new mrob::NodePose2d(x));
    graph.add_node(n1);
    Mat3 obsInformation= Mat3::Identity();
    std::shared_ptr<mrob::Factor> f1(new mrob::Factor1Pose2d(obs,n1,obsInformation*1e6));
    graph.add_factor(f1);



    // Node 2, initialized at 0,0,0
    std::shared_ptr<mrob::Node> n2(new mrob::NodePose2d(x));
    graph.add_node(n2);

    // Add odom factor = [drot1, dtrans, drot2]
    obs << 0, 1, 0;
    //obs << M_PI_2, 0.5, 0;
    // this factor assumes that the current value of n2 (node destination) is updated according to obs
    std::shared_ptr<mrob::Factor> f2(new mrob::Factor2Poses2dOdom(obs,n1,n2,obsInformation));
    graph.add_factor(f2);

    obs << -1 , -1 , 0;
    std::shared_ptr<mrob::Factor> f3(new mrob::Factor2Poses2d(obs,n2,n1,obsInformation));
    graph.add_factor(f3);




    // solve the Gauss Newton optimization
    graph.print(true);
    graph.solve_batch();

    std::cout << "\nSolved, chi2 = " << graph.evaluate_problem() << std::endl;

    graph.print(true);
    return 0;
}
