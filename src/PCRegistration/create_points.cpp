/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * create_points.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */


#include "mrob/create_points.hpp"

#include <chrono>
#include <assert.h>
#include <iostream>
#include <cmath>

using namespace mrob;

CsampleUniformSE3::CsampleUniformSE3(double R_range, double t_range):
    R_uniform_(-R_range, R_range), t_uniform_(-t_range, t_range)
{
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.seed(seed);
}

CsampleUniformSE3::CsampleUniformSE3(double R_min, double R_max, double t_min, double t_max):
    R_uniform_(R_min, R_max), t_uniform_(t_min, t_max)
{
    assert(R_min <= R_max && "\nCsampleUniformSE3::CsampleUniformSE3 incorrect R bounds");
    assert(t_min <= t_max && "\nCsampleUniformSE3::CsampleUniformSE3 incorrect t bounds");
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.seed(seed);
}


CsampleUniformSE3::~CsampleUniformSE3()
{
}

SE3 CsampleUniformSE3::samplePose()
{
    // xi = [w , v]^T
    Mat61 xi;
    xi <<   R_uniform_(generator_),
            R_uniform_(generator_),
            R_uniform_(generator_),
            t_uniform_(generator_),
            t_uniform_(generator_),
            t_uniform_(generator_);
    return SE3(xi);
}


Mat31 CsampleUniformSE3::samplePosition()
{
    return Mat31(t_uniform_(generator_), t_uniform_(generator_), t_uniform_(generator_));
}

SO3 CsampleUniformSE3::sampleOrientation()
{
    Mat31 w(R_uniform_(generator_), R_uniform_(generator_), R_uniform_(generator_));
    return SO3(w);
}

std::vector<Mat31>& CreatePoints::get_point_cloud(uint_t t)
{
    assert(t < numberPoses_ && "CreatePoints::getPointCloud: temporal index larger than number of calculated poses\n");
    return X_[t];//.at(t);
}

CreatePoints::CreatePoints(uint_t N, uint_t numberPlanes, double noisePerPoint):
        N_(10),
        numberPlanes_(numberPlanes),
        noisePerPoint_(noisePerPoint),
        R_range_(M_PI),
        t_range_(10.0),
        lamdaOutlier_(0.0),
        samplePoses_(R_range_,t_range_),
        // Trajectory parameters
        xRange_(10.0),
        yRange_(10.0),
        numberPoses_(6)
{
    // 1) generate planes
    for (uint_t i = 0; i < numberPlanes_ ; ++i)
    {

    }
    // 2) generate trajectory, from x_o = 0,,..,0, to x_f = at xRange, yRange, z
    // 3) generate points for each plane
}

CreatePoints::~CreatePoints()
{

}


void CreatePoints::sample_plane(uint_t nPoints, uint_t id, uint_t t)
{
    for (uint_t i = 0; i < nPoints; ++i)
    {
        // generate point
        // transform point
        // push_back to the corresponding instant of time
    }
}
