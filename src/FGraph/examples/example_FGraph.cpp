/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * example_FGraph.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include <iostream> //this causes 1 non-free allocation in valgrind, dont panic

#include "mrob/factor_graph.hpp"
#include "mrob/factors/factor2Poses3d.hpp"
#include "mrob/factors/nodePose3d.hpp"


// TODO remove soon

int main()
{
    // create a simple FGraph testing the insertion and deletions of elements
    mrob::FGraph fgr(50,50);

    //add nodes. In general Nodes just need their initial state.
    Mat61 xIni;
    xIni << 0,0.1,0,1,-3,2;
    std::shared_ptr<mrob::Node> n(new mrob::NodePose3d(xIni));
    fgr.addNode(n);
    std::shared_ptr<mrob::Node> n2(new mrob::NodePose3d(xIni));
    fgr.addNode(n2);

    // add factors. We need to specify the node/nodes connecting the factor
    Mat6 obsCov = Mat6::Identity();
    std::shared_ptr<mrob::Factor> f(new mrob::Factor2Poses3d(xIni,n,n2,obsCov));
    fgr.addFactor(f);

    fgr.print(1);

    return 1;
}
