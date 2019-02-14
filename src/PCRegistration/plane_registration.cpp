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


PlaneRegistration::PlaneRegistration():
        isSolved_(0), trajectory_(new std::vector<SE3>(8, SE3()))
{
}

PlaneRegistration::PlaneRegistration(uint_t numberPlanes, uint_t numberPoses):
        isSolved_(0), trajMode_(TrajectoryMode::SEQUENCE), trajectory_(new std::vector<SE3>(numberPoses, SE3()))
{
    planes_.reserve(numberPlanes);
}

PlaneRegistration::~PlaneRegistration()
{

}


void PlaneRegistration::set_number_planes_and_poses(uint_t numberPlanes, uint_t numberPoses)
{
    planes_.reserve(numberPlanes);
    trajectory_->reserve(numberPoses);
}

uint_t PlaneRegistration::solve()
{
    // TODO iterative process, on what convergence basis?
    {

        // 1) calculate plane estimation given the current trajectory
        double  error = 0.0;
        for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
        {
                it->second->estimate_plane();
                error += it->second->get_error();
        }

        // 2) calculate Jacobians
        Mat61 jacobian;
        double  numberPoints;
        // starts at t = 1 because t = 0 is the fixed reference frame t0 = I
        for (uint_t t = 1 ; t < trajectory_->size(); ++t)
        {
            jacobian.setZero();
            numberPoints = 0.0;
            for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
            {
                jacobian += it->second->calculate_jacobian(t);
                numberPoints += it->second->get_number_points(t);
            }

            // 3) update results Ti = exp(-dxi) * Ti (our convention, we expanded from the left)
            // 3.1) first atempt: gradient decent alpha = 1/N
            trajectory_->at(t).update(-jacobian/(2*numberPoints));

            // 3.2) line search
            // 3.3) SR1
        }
        std::cout << "error =                                                 " << error << std::endl;
    }

    return 1;
}

void PlaneRegistration::add_plane(uint_t id, std::shared_ptr<Plane> &plane)
{
    plane->set_trajectory(trajectory_);
    planes_.emplace(id, plane);
}

void PlaneRegistration::print(bool plotPlanes) const
{
    std::cout << "Printing plane registration data :"<< std::endl;
    for (SE3 &transf : *trajectory_)
        transf.print();
    if (plotPlanes)
    {
        for (auto it = planes_.cbegin();  it != planes_.cend(); ++it)
            it->second->print();
    }
}
