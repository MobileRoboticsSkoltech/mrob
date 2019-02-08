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

SampleUniformSE3::SampleUniformSE3(double R_range, double t_range):
        rotationUniform_(-R_range, R_range), tUniform_(-t_range, t_range)
{
    assert(R_range <= 0.0 && "\nSampleUniformSE3::SampleUniformSE3 incorrect R bounds");
    assert(t_range <= 0.0 && "\nSampleUniformSE3::SampleUniformSE3 incorrect t bounds");
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.seed(seed);
}

SampleUniformSE3::SampleUniformSE3(double R_min, double R_max, double t_min, double t_max):
    rotationUniform_(R_min, R_max), tUniform_(t_min, t_max)
{
    assert(R_min <= R_max && "\nSampleUniformSE3::SampleUniformSE3 incorrect R bounds");
    assert(t_min <= t_max && "\nSampleUniformSE3::SampleUniformSE3 incorrect t bounds");
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.seed(seed);
}


SampleUniformSE3::~SampleUniformSE3()
{
}

SE3 SampleUniformSE3::samplePose()
{
    // xi = [w , v]^T
    Mat61 xi;
    xi <<   rotationUniform_(generator_),
            rotationUniform_(generator_),
            rotationUniform_(generator_),
            tUniform_(generator_),
            tUniform_(generator_),
            tUniform_(generator_);
    return SE3(xi);
}


Mat31 SampleUniformSE3::samplePosition()
{
    return Mat31(tUniform_(generator_), tUniform_(generator_), tUniform_(generator_));
}

SO3 SampleUniformSE3::sampleOrientation()
{
    Mat31 w(rotationUniform_(generator_), rotationUniform_(generator_), rotationUniform_(generator_));
    return SO3(w);
}

SamplePlanarSurface::SamplePlanarSurface(double zStd):
        x_(-1.0,1.0), y_(-1.0,1.0), z_(0.0,zStd)
{
    assert(zStd <= 0.0 && "\nSamplePlanarSurface::SamplePlanarSurfaceincorrect z bounds");
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.seed(seed);
}

SamplePlanarSurface::~SamplePlanarSurface()
{
}

Mat31 SamplePlanarSurface::samplePoint(double length)
{

    return Mat31(x_(generator_), y_(generator_), z_(generator_) );
}

std::vector<Mat31>& CreatePoints::get_point_cloud(uint_t t)
{
    assert(t < numberPoses_ && "CreatePoints::getPointCloud: temporal index larger than number of calculated poses\n");
    return X_[t];//.at(t);
}

CreatePoints::CreatePoints(uint_t N, uint_t numberPlanes, double noisePerPointStd):
        N_(10),
        numberPlanes_(numberPlanes),
        noisePerPoint_(noisePerPointStd),
        rotationRange_(M_PI),
        transRange_(10.0),
        lamdaOutlier_(0.0),
        samplePoses_(rotationRange_,transRange_),
        samplePoints_(noisePerPoint_),
        // Trajectory parameters
        xRange_(10.0),
        yRange_(10.0),
        numberPoses_(6)
{
    // 0) initialize vectors and variables
    X_.reserve(numberPoses_);
    pointId_.reserve(numberPoses_);
    poseGroundTruth_.reserve(numberPoses_);
    planes_.reserve(numberPlanes_);
    for (uint_t i = 0; i < numberPoses_; ++i)
    {
        X_[i].reserve(N_);
        pointId_[i].reserve(N_);
    }


    // 1) generate planes
    for (uint_t i = 0; i < numberPlanes_ ; ++i)
    {
        planes_.push_back(samplePoses_.samplePose());
    }

    // 2) generate initial and final pose, TODO We could add more intermediate points
    initialPose_ = samplePoses_.samplePose();
    SE3 initialPoseInv = initialPose_.inv();
    finalPose_  = samplePoses_.samplePose();
    SE3 dx =  finalPose_ * initialPoseInv;
    Mat61 dxi = dx.ln_vee();

    // 2.1 generate trajectory
    for (uint_t t = 0; t < numberPoses_ ; ++t)
    {
        // proper interpolation in SE3 is exp( t * ln( T1*T0^-1) )T0
        // (1-t)ln(T0) + t ln(T1) only works if |dw| < pi, which we can guarantee on these sampling conditions
        Mat61 tdx =  double(t) / double(numberPoses_-1) * dxi;
        SE3 pose = SE3( tdx ) * initialPose_;
        poseGroundTruth_.push_back(pose);
    }

    // 3) generate points for each plane
    for (uint_t t = 0; t < numberPoses_ ; ++t)
    {
        // prepare permutation of points
        for (uint_t i = 0; i < numberPlanes_ ; ++i)
        {
            //X_[t].push_back( planes_[i].transform( samplePoints_.samplePoint( 2.0 ) ) );
            pointId_[t].push_back(i);
        }
    }
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
