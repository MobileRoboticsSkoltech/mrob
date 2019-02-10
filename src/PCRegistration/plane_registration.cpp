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
        isSolved_(0)
{
    planes_.reserve(numberPlanes);
    transformations_.reserve(numberPoses);
}

PlaneRegistration::~PlaneRegistration()
{

}


void PlaneRegistration::print() const
{
    std::cout << "Printing plane registration data :"<< std::endl;
    for (SE3 transf : transformations_)
        transf.print();
    for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
        it->second->print();
}
