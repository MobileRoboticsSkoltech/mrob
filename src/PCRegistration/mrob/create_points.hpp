/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * create_points.hpp
 *
 *  Created on: Feb 6, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef CREATE_POINTS_HPP_
#define CREATE_POINTS_HPP_

#include "mrob/SE3.hpp"
#include <random>
#include <Eigen/StdVector> // for fixed size SE3 objects


using namespace Eigen;

namespace mrob{

/**
 * Class samples a random configuration on SE3 using a
 * uniform distribution U(-R_range, R_range)
 * or any implemented distribution
 */
class CsampleUniformSE3{
  public:
    CsampleUniformSE3(double R_range, double t_range);
    CsampleUniformSE3(double R_min, double R_max, double t_min, double t_max);
    ~CsampleUniformSE3();
    SE3 samplePose();
    Mat31 samplePosition();
    SO3 sampleOrientation();
  protected:
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> R_uniform_;
    std::uniform_real_distribution<double> t_uniform_;
};




/**
 * Class generating a sequence of pointClouds given some specifications.
 */
class CreatePoints{
public:
    /**
     * Creates a class
     */
    CreatePoints(uint_t N = 1000, uint_t numberPlanes = 4, double noisePerPoint = 1.0);
    ~CreatePoints();

    std::vector<Mat31>& get_point_cloud(uint_t t);
    std::vector<int>& get_plane_ids(uint_t t);


protected:
    // generation parameters
    uint_t N_; // Number of points
    uint_t numberPlanes_; // Number of planes in the virtual environment
    double noisePerPoint_;
    double R_range_;
    double t_range_;
    double lamdaOutlier_;
    CsampleUniformSE3 samplePoses_;

    // Point cloud data generated
    std::vector< std::vector<Mat31> > X_;
    // IDs for facilitating the task of DA and normal computation
    std::vector< std::vector<uint_t>> pointId_;

    // Trajectory parameters
    double xRange_, yRange_; // dimension of the workspace
    SE3 initialPose, finalPose;
    std::vector<SE3> poseGroundTruth_;
    uint_t numberPoses_;

    // Generation of planes
    void sample_plane(uint_t nPoints, uint_t id, uint_t t);
    std::vector<SE3> planes_;

};

}
#endif /* CREATE_POINTS_HPP_ */
