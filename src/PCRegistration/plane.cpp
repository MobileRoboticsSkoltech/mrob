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
#include <iostream>



using namespace mrob;

Plane::Plane(uint_t timeLength): timeLength_(timeLength)
{
    allPlanePoints_.reserve(timeLength_);
    // there should be an indivudual reversation of points
    for (uint_t i = 0; i < timeLength_; ++i)
    {
        // estimated number of points per observation TODO
        allPlanePoints_[i].reserve(512);
    }
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
    // XXX for now it ony works on consecutives observations TODO
    if (t < timeLength_)
    {
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

void Plane::print() const
{
    for (uint_t t = 0; t <  timeLength_; ++t)
    {
        std::cout << "Plane time = " << t << std::endl;
        for (Mat31 p : allPlanePoints_[t])
            std::cout << p.transpose() << std::endl;
    }
}
