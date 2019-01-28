//
// Created by Konstantin on 14/01/2019.
//

#include "mrob/factor_graph.hpp"
#include "mrob/factors/factor2Poses2d.h"
#include "mrob/factors/nodePose2d.h"

int main()
{
    // create a simple FGraph testing the insertion and deletions of elements
    mrob::FGraph fgr(50,50);

    Mat31 xIni;
    xIni << 180, 50, 0;

    std::shared_ptr<mrob::Node> n(new mrob::NodePose2d(xIni)), n2(new mrob::NodePose2d(xIni));

    fgr.addNode(n);
    fgr.addNode(n2);

    // add factors. We need to specify the node/nodes connecting the factor
    Mat3 obsCov = Mat3::Identity();
    std::shared_ptr<mrob::Factor> f(new mrob::Factor2Poses2d(xIni,n,n2,obsCov));

    fgr.addFactor(f);

    fgr.print(true);

    return 0;
}
