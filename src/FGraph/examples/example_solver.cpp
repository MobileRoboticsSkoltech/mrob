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





#include "solverDense.hpp"
#include "nodePose3d.hpp"
#include "factor1Pose3d.hpp"


int main ()
{

    // create a simple graph to solve: 1 node multiple anchor observations
    std::shared_ptr<fg::FGraph> graph(new fg::FGraph(1,50));
    Mat61 xIni, obs;
    xIni = Mat61::Zero();
    std::shared_ptr<fg::Node> n1(new fg::NodePose3d(xIni));
    graph->addNode(n1);
    Mat6 obsCov = Mat6::Identity();
    for (auto i = 0; i < 3; ++i)
    {
        obs = Mat61::Random();
        std::shared_ptr<fg::Factor> f(new fg::Factor1Pose3d(obs,n1,obsCov));
        graph->addFactor(f);
    }
    graph->print();

    // solve the Gauss Newton optimization
    fg::DenseGaussNewton problem(graph);
    problem.solveOnce();

    return 0;
}
