/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * plane.cpp
 *
 *  Created on: Feb 1, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */



#include "mrob/plane.hpp"



using namespace mrob;

Plane::Plane(uint_t timeLength): timeLength_(timeLength)
{
    allPlanePoints_.reserve(timeLength_);
    // there should be an indivudual reversation of points
}

Plane::~Plane()
{
}

void Plane::reserve(uint_t d, uint_t t)
{
    if (t < timeLength_)
        allPlanePoints_[t].reserve(d);
}

void Plane::set_plane(SE3 &T)
{
    plane_ = T;
}

SE3 Plane::get_plane(void)
{
    return plane_;
}

void Plane::push_back_point(Mat31 &point, uint_t t)
{

    if (t < timeLength_)
    {
        //TODO why homogeneous?
        //Mat41 homogeneousPoint;
        //homogeneousPoint << point, 1.0;
        //allPlanePoints_[t].push_back(homogeneousPoint);
        allPlanePoints_[t].push_back(point);
    }
}

std::vector<Mat31>& Plane::get_points(uint_t t)
{
    if (t >= timeLength_)
        return allPlanePoints_.back();
    return allPlanePoints_.at(t);
}

void Plane::clear_points()
{
    allPlanePoints_.clear();
}
