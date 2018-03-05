/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * test_FGraph.cpp
 *
 *  Created on: Feb 28, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "FGraph.hpp"
#include "nodePose3d.hpp"
#include "factor2Poses3d.hpp"
#include <iostream> //this causes 1 non-free allocation in valgrind, dont panic


int main()
{
    // create a simple FGraph testing the insertion and deletions of elements
    fg::FGraph fgr(50,50);

    //add nodes. In general Nodes just need their initial state.
    Mat61 xIni;
    xIni << 0,0.1,0,1,-3,2;
    std::shared_ptr<fg::Node> n(new fg::NodePose3d(xIni));
    fgr.addNode(n);
    std::shared_ptr<fg::Node> n2(new fg::NodePose3d(xIni));
    fgr.addNode(n2);

    // add factors. We need to specify the node/nodes connecting the factor
    Mat6 obsCov = Mat6::Identity();
    std::shared_ptr<fg::Factor> f(new fg::Factor2Poses3d(xIni,n,n2,obsCov));
    fgr.addFactor(f);

    fgr.rmNode(n);
    fgr.rmFactor(f);
    fgr.print(1);

    return 1;
}
