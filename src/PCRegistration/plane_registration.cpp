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
#include <iostream>

using namespace mrob;

PlaneRegistration::PlaneRegistration(uint_t numberPlanes, uint_t numberPoses):
        isSolved_(0), trajectory_(new std::vector<SE3>(numberPoses, SE3()))
{
    planes_.reserve(numberPlanes);
    //trajectory_->reserve();
}

PlaneRegistration::~PlaneRegistration()
{

}

void PlaneRegistration::add_plane(uint_t id, std::shared_ptr<Plane> &plane)
{
    plane->set_trajectory(trajectory_);
    planes_.emplace(id, plane);
}

void PlaneRegistration::print() const
{
    std::cout << "Printing plane registration data :"<< std::endl;
    for (SE3 &transf : *trajectory_)
        transf.print();
    for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
        it->second->print();
}
