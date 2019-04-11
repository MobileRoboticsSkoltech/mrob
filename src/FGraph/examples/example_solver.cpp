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


int main ()
{

    // create a simple graph to solve: 1 node multiple anchor observations
    mrob::FGraphSolve graph(mrob::FGraphSolve::CHOL_ADJ,50,50);
    Mat31 xIni, obs;
    xIni = Mat31::Zero();
    std::shared_ptr<mrob::Node> n1(new mrob::NodePose2d(xIni));
    graph.addNode(n1);
    std::shared_ptr<mrob::Node> n2(new mrob::NodePose2d(xIni));
    graph.addNode(n2);
    Mat3 obsCov = Mat3::Identity();
    obsCov = obsCov*obsCov.transpose();
    obs << 1 , 1 , 0;
    std::shared_ptr<mrob::Factor> f1(new mrob::Factor1Pose2d(obs,n1,obsCov));
    graph.addFactor(f1);
    for (auto i = 0; i < 2; ++i)
    {
        obs = Mat31::Random();
        std::shared_ptr<mrob::Factor> f2(new mrob::Factor2Poses2d(obs,n1,n2,obsCov));
        graph.addFactor(f2);
    }



    graph.print(true);

    // solve the Gauss Newton optimization
    graph.buildProblem();
    graph.solveOnce();

    graph.print(true);
    return 0;
}
