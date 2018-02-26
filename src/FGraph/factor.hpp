/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * factor.hpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR_HPP_
#define FACTOR_HPP_

#include "node.hpp"
#include "Eigen/Dense"

namespace skmr{

class Factor{
public:
    Factor();
    virtual ~Factor() = 0;
    virtual void evaluate(void) = 0;
    virtual void evaluateJacobians(void) = 0;
protected:
    Eigen::VectorXd obs_; //will this caouse any problem?
    //std::vector<skmr::Node*> nodes_;
};

}

#endif /* FACTOR_HPP_ */
