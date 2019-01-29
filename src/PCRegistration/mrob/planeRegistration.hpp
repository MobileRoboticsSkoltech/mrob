/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * planeRegistration.hpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef PLANEREGISTRATION_HPP_
#define PLANEREGISTRATION_HPP_


#include "mrob/SE3.hpp"


namespace mrob{


class Plane{
  public:
    Plane(SE3);
    ~Plane();

  protected:
    // Overparametrized plane, as a transformation
    SE3 plane_;
    // for now a vector, is it necessary a queue to remove data?
    std::vector<MatX> data_;
};





}// namespace
#endif /* PLANEREGISTRATION_HPP_ */
