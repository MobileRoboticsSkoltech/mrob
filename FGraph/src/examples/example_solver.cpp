/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * example_solver.cpp
 *
 *  Created on: Mar 22, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */





#include "../../include/skmr/factor_graph_solve.hpp"
#include "skmr/factors/nodePose3d.hpp"
#include "skmr/factors/factor1Pose3d.hpp"
#include "skmr/factors/factor2Poses3d.hpp"


int main ()
{

    // create a simple graph to solve: 1 node multiple anchor observations
    skmr::FGraphSolve graph(skmr::FGraphSolve::CHOL_ADJ,50,50);
    Mat61 xIni, obs;
    xIni = Mat61::Zero();
    std::shared_ptr<skmr::Node> n1(new skmr::NodePose3d(xIni));
    graph.addNode(n1);
    std::shared_ptr<skmr::Node> n2(new skmr::NodePose3d(xIni));
    graph.addNode(n2);
    Mat6 obsCov = Mat6::Random();
    obsCov = obsCov*obsCov.transpose();
    std::shared_ptr<skmr::Factor> f1(new skmr::Factor1Pose3d(obs,n1,obsCov));
    graph.addFactor(f1);
    for (auto i = 0; i < 2; ++i)
    {
        obs = Mat61::Random();
        std::shared_ptr<skmr::Factor> f2(new skmr::Factor2Poses3d(obs,n1,n2,obsCov));
        graph.addFactor(f2);
    }



    graph.print(1);

    // solve the Gauss Newton optimization
    graph.buildProblem();

    return 0;
}
