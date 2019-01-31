/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * planeRegistration.cpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include "mrob/plane_registration.hpp"

using namespace mrob;

Plane::Plane(uint_t timeLength): timeLength_(timeLength)
{
    points_.reserve(timeLength_);

}

Plane::~Plane()
{
}

void Plane::set_plane(SE3 &T)
{
    plane_ = T;
}

SE3 Plane::get_plane(void)
{
    return plane_;
}
