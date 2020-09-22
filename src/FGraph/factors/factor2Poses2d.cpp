/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * See LICENSE file in the root of the mrob library.
 *
 *
 *  Created on: Jan 14, 2019
 *      Author: Konstantin Pakulev
 *              konstantin.pakulev@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include <iostream>
#include <mrob/factors/factor2Poses2d.hpp>


using namespace mrob;


Factor2Poses2d::Factor2Poses2d(const Mat31 &observation, std::shared_ptr<Node> &nodeOrigin,
                               std::shared_ptr<Node> &nodeTarget, const Mat3 &obsInf, bool updateNodeTarget):
        Factor(3, 6), obs_(observation), W_(obsInf)
{
    if (nodeOrigin->get_id() < nodeTarget->get_id())
    {
        neighbourNodes_.push_back(nodeOrigin);
        neighbourNodes_.push_back(nodeTarget);
    }
    else
    {
        // we reverse the order and simply invert the observation function (not always true)
        neighbourNodes_.push_back(nodeTarget);
        neighbourNodes_.push_back(nodeOrigin);

        // reverse observations to account for this
        obs_ = -observation;
    }
    if (updateNodeTarget)
    {
        Mat31 dx = nodeOrigin->get_state() +  obs_ - nodeTarget->get_state();
        nodeTarget->update(dx);
    }
}


void Factor2Poses2d::evaluate_residuals() {
    // Evaluation of h(i,j)
    Mat31   nodeOrigin = get_neighbour_nodes()->at(0).get()->get_state(),
            nodeTarget = get_neighbour_nodes()->at(1).get()->get_state();

    // r = h(i,j) - obs = Ri^T * (xj- xi) - obs .  From "i", i.e, at its reference frame, we observe xj
    Mat31 h = nodeTarget - nodeOrigin;
    Mat2 RiT;
    double c1 = cos(nodeOrigin(2)),
           s1 = sin(nodeOrigin(2));
    RiT <<  c1, s1,
           -s1, c1;
    h.head(2) = RiT * h.head(2);
    r_ = h - obs_;
    r_(2) = wrap_angle(r_(2));
}

void Factor2Poses2d::evaluate_jacobians()
{
    Mat31   nodeOrigin = get_neighbour_nodes()->at(0).get()->get_state(),
            nodeTarget = get_neighbour_nodes()->at(1).get()->get_state();

    // r =  Ri^T * (xj- xi) - obs
    // J1 = [-R1^T,    J2 = [R1^T 0]
    //      [  ]           [0    1]
    double c1 = cos(nodeOrigin(2)),
           s1 = sin(nodeOrigin(2)),
           dx = nodeTarget(0) - nodeOrigin(0),
           dy = nodeTarget(1) - nodeOrigin(1);
    J_ <<   -c1, -s1, -s1*dx + c1*dy,     c1, s1, 0,
             s1, -c1, -c1*dx - s1*dy,    -s1, c1, 0,
              0,   0,    -1,              0,  0, 1;
}

void Factor2Poses2d::evaluate_chi2()
{
    chi2_ = 0.5 * r_.dot(W_ * r_);
}

void Factor2Poses2d::print() const
{
    std::cout << "Printing Factor:" << id_ << ", obs= \n" << obs_
              << "\n Residuals=\n " << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = \n" << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}


Factor2Poses2dOdom::Factor2Poses2dOdom(const Mat31 &observation, std::shared_ptr<Node> &nodeOrigin, std::shared_ptr<Node> &nodeTarget,
                         const Mat3 &obsInf, bool updateNodeTarget) :
                         Factor2Poses2d(observation, nodeOrigin, nodeTarget, obsInf)
{
    assert(nodeOrigin->get_id() < nodeTarget->get_id() && "Factor2Poses2dOdom::Factor2Poses2dodom: Node origin id is posterior to the destination node\n");
    if (updateNodeTarget)
    {
        Mat31 dx = get_odometry_prediction(nodeOrigin->get_state(), obs_) - nodeTarget->get_state();
        nodeTarget->update(dx);
    }
}

void Factor2Poses2dOdom::evaluate_residuals()
{
    // Evaluation of residuals as g (x_origin, observation) - x_dest
    auto    stateOrigin = get_neighbour_nodes()->at(0).get()->get_state(), // x[i - 1]
            stateTarget = get_neighbour_nodes()->at(1).get()->get_state(); // x[i]
    auto prediction = get_odometry_prediction(stateOrigin, obs_);

    r_ =  prediction - stateTarget;
    r_[2] = wrap_angle(r_[2]);

}
void Factor2Poses2dOdom::evaluate_jacobians()
{
    // Get the position of node we are traversing from
    Mat31 node1 = get_neighbour_nodes()->at(0).get()->get_state();

    auto s = -obs_[1] * sin(node1[2]), c = obs_[1] * sin(node1[2]);

    // Jacobians for odometry model which are: G and -I
    J_ <<   1, 0, s,    -1, 0, 0,
            0, 1, c,    0, -1, 0,
            0, 0, 1,    0, 0, -1;
}


Mat31 Factor2Poses2dOdom::get_odometry_prediction(Mat31 state, Mat31 motion) {
    state[2] += motion[0];
    state[0] += motion[1] * cos(state[2]);
    state[1] += motion[1] * sin(state[2]);
    state[2] += motion[2];

    return state;
}
