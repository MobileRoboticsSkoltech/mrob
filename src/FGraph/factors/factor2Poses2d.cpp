/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 *  Created on: Jan 14, 2019
 *      Author: Konstantin Pakulev
 *              konstantin.pakulev@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include <iostream>
#include <Eigen/Cholesky>
#include <mrob/factors/factor2Poses2d.hpp>


using namespace mrob;


Factor2Poses2d::Factor2Poses2d(const Mat31 &observation, std::shared_ptr<Node> &n1,
                               std::shared_ptr<Node> &n2, const Mat3 &obsInf):
        Factor(3, 6), obs_(observation), W_(obsInf)
{
    assert(n1->get_id() && "Factor2Poses2d::Factor2Poses2d: Non initialized Node1. Add nodes first and then Factors to the FG\n");
    assert(n2->get_id() && "Factor2Poses2d::Factor2Poses2d: Non initialized Node2. Add nodes first and then Factors to the FG\n");
    if (n1->get_id() < n2->get_id())
    {
        neighbourNodes_.push_back(n1);
        neighbourNodes_.push_back(n2);
    }
    else
    {
        neighbourNodes_.push_back(n2);
        neighbourNodes_.push_back(n1);
    }
    WT2_ = W_.llt().matrixU();
}



void Factor2Poses2d::evaluate() {
    // residuals
    this->evaluate_error();

    // Jacobians
    J_ <<   1, 0, 0, -1, 0, 0,
            0, 1, 0, 0, -1, 0,
            0, 0, 1, 0, 0, -1;
}

matData_t Factor2Poses2d::evaluate_error() {
    // Evaluation of h(i,j)
    auto    node1 = get_neighbour_nodes()->at(0).get()->get_state(),
            node2 = get_neighbour_nodes()->at(1).get()->get_state();

    auto h = node2 - node1;

    r_ = h - obs_;
    r_[2] = wrap_angle(r_[2]);

    return 0.0;
}

void Factor2Poses2d::print() const
{
    std::cout << "Printing Factor: " << id_ << ", obs= \n" << obs_
              << "\n Residuals= " << r_
              << " \nand Information matrix\n" << W_
              << "\n Calculated Jacobian = " << J_
              << "\n Chi2 error = " << chi2_
              << " and neighbour Nodes " << neighbourNodes_.size()
              << std::endl;
}

// TODO move me to a genral place for nodes as well
double Factor2Poses2d::wrap_angle(double angle) {
    double pi2 = 2 * M_PI;

    while (angle < -M_PI) angle += pi2;
    while (angle >= M_PI) angle -= pi2;

    return angle;
}

Factor2Poses2dOdom::Factor2Poses2dOdom(const Mat31 &observation, std::shared_ptr<Node> &n1, std::shared_ptr<Node> &n2,
                                     const Mat3 &obsInf) : Factor2Poses2d(observation, n1, n2, obsInf)
{
}

void Factor2Poses2dOdom::evaluate()
{
    // residuals
    this->evaluate_error();

    // Get the position of node we are traversing from
    auto node1 = get_neighbour_nodes()->at(0).get()->get_state();

    auto s = -obs_[1] * sin(node1[2]), c = obs_[1] * sin(node1[2]);

    // Jacobians for odometry model which are: G and -I
    J_ <<   1, 0, s,    -1, 0, 0,
            0, 1, c,    0, -1, 0,
            0, 0, 1,    0, 0, -1;
}

matData_t Factor2Poses2dOdom::evaluate_error()
{
    // Evaluation of residuals as x[i] - f(x[i - 1], u[i])
    auto    node1 = get_neighbour_nodes()->at(0).get()->get_state(), // x[i - 1]
            node2 = get_neighbour_nodes()->at(1).get()->get_state(); // x[i]
    auto prediction = get_odometry_prediction(node1, obs_);

    r_ = node2 - prediction;
    r_[2] = wrap_angle(r_[2]);

    return 0.0;
}

Mat31 Factor2Poses2dOdom::get_odometry_prediction(Mat31 state, Mat31 motion) {
    state[2] += motion[0];
    state[0] += motion[1] * cos(state[2]);
    state[1] += motion[1] * sin(state[2]);
    state[2] += motion[2];

    return state;
}

