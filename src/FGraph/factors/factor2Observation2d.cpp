//
// Created by Konstantin on 17/01/2019.
//

#include <iostream>
#include <mrob/factors/factor2Observation2d.hpp>

using namespace mrob;


Factor2Observation2d::Factor2Observation2d(const Mat31 &observation, std::shared_ptr<Node> &n1,
                                           std::shared_ptr<Node> &n2, const Mat3 &obsInf) :
                                           Factor2Poses2d(observation, n1, n2, obsInf) {
}

void Factor2Observation2d::evaluate() {
    // residuals
    this->evaluateError();

    // Jacobians
    J_ <<   1, 0, 0, -1, 0, 0,
            0, 1, 0, 0, -1, 0,
            0, 0, 1, 0, 0, -1;
}

matData_t Factor2Observation2d::evaluateError() {
    // Evaluation of h(i,j)
    auto    node1 = getNeighbourNodes()->at(0).get()->getState(),
            node2 = getNeighbourNodes()->at(1).get()->getState();

    auto h = node2 - node1;

    r_ = h - obs_;
    r_[2] = wrap_angle(r_[2]);

    return 0.0;
}


