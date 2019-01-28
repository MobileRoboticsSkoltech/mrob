//
// Created by Konstantin on 17/01/2019.
//

#include <mrob/factors/factor2Odometry2d.h>

using namespace mrob;


Factor2Odometry2d::Factor2Odometry2d(const Mat31 &observation, std::shared_ptr<Node> &n1, std::shared_ptr<Node> &n2,
                                     const Mat3 &obsInf) : Factor2Poses2d(observation, n1, n2, obsInf) {
}

void Factor2Odometry2d::evaluate()
{
    // residuals
    this->evaluateError();

    // Get the position of node we are traversing from
    auto node1 = getNeighbourNodes()->at(0).get()->getState();

    auto s = -obs_[1] * sin(node1[2]), c = obs_[1] * sin(node1[2]);

    // Jacobians for odometry model which are: G and -I
    J_ <<   1, 0, s,    -1, 0, 0,
            0, 1, c,    0, -1, 0,
            0, 0, 1,    0, 0, -1;
}
matData_t Factor2Odometry2d::evaluateError()
{
    // Evaluation of residuals as x[i] - f(x[i - 1], u[i])
    auto    node1 = getNeighbourNodes()->at(0).get()->getState(), // x[i - 1]
            node2 = getNeighbourNodes()->at(1).get()->getState(); // x[i]
    auto prediction = get_odometry_prediction(node1, obs_);

    r_ = node2 - prediction;
    r_[2] = wrap_angle(r_[2]);

    return 0.0;
}

Mat31 Factor2Odometry2d::get_odometry_prediction(Mat31 state, Mat31 motion) {
    state[2] += motion[0];
    state[0] += motion[1] * cos(state[2]);
    state[1] += motion[1] * sin(state[2]);
    state[2] += motion[2];

    return state;
}


