/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
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
#include "mrob/plane.hpp"
#include "mrob/plane_registration.hpp"
#include <random>
#include <memory>
#include <utility>
#include <Eigen/StdVector> // for fixed size SE3 objects



using namespace Eigen;

namespace mrob{

/**
 * Class samples a random configuration on SE3 using a
 * uniform distribution U(-R_range, R_range)
 * or any implemented distribution
 */
class SampleUniformSE3{
  public:
    SampleUniformSE3(double R_range, double t_range);
    SampleUniformSE3(double R_min, double R_max, double t_min, double t_max);
    ~SampleUniformSE3();
    SE3 samplePose();
    Mat31 samplePosition();
    SO3 sampleOrientation();
  protected:
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> rotationUniform_;
    std::uniform_real_distribution<double> tUniform_;
};

/**
 * Class samples a point on a surface over the plane XY
 * according to a fixed noise on height
 */
class SamplePlanarSurface{
  public:
    SamplePlanarSurface(double zStd, double bias = 0.0);
    ~SamplePlanarSurface();
    /**
     * samples a point with noise on a square of given length
     */
    Mat31 samplePoint(double length);
    void sampleBias();

  protected:
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> x_, y_;
    std::normal_distribution<double> z_, bias_;
    double xBias_, yBias_;
};

/**
 * Class generating a sequence of pointClouds given some specifications.
 */
class CreatePoints{
public:
    /**
     * Creates a class
     */
    CreatePoints(uint_t N = 10, uint_t numberPlanes = 4, uint_t numberPoses = 2, double noisePerPoint = 0.01, double noiseBias = 0.1);
    ~CreatePoints();

    /**
     * Fill the Planeregistration class with planes calcualted here (reset)
     * and a new initial traejctory set to I's
     */
    void create_plane_registration(PlaneRegistration& planeReg);

    std::vector<Mat31>& get_point_cloud(uint_t t);
    std::vector<uint_t>& get_point_plane_ids(uint_t t);

    uint_t get_number_planes() const {return numberPlanes_;};
    uint_t get_number_poses() const {return numberPoses_;};

    std::vector<SE3>& get_ground_truth_trajectory() {return goundTruthTrajectory_;};

    std::vector<SE3>& get_plane_poses() {return planePoses_;};
    std::vector<std::pair<uint_t, std::shared_ptr<Plane> >>& get_all_planes() {return planes_;}

    void print() const;


protected:
    // generation parameters
    uint_t numberPoints_; // Number of points
    uint_t numberPlanes_; // Number of planes in the virtual environment
    double noisePerPoint_, noiseBias_;
    double rotationRange_;
    double transRange_;
    double lamdaOutlier_;
    SampleUniformSE3 samplePoses_,samplePlanes_;
    SamplePlanarSurface samplePoints_;

    // Point cloud data generated
    std::vector< std::vector<Mat31> > X_;
    // IDs for facilitating the task of DA and normal computation
    std::vector< std::vector<uint_t>> pointId_;

    // Trajectory parameters
    double xRange_, yRange_; // dimension of the workspace
    SE3 initialPose_, finalPose_;
    std::vector<SE3> goundTruthTrajectory_;// ground truth trajectory
    uint_t numberPoses_;

    // Generation of planes
    std::vector<SE3> planePoses_;
    std::vector<std::pair<uint_t, std::shared_ptr<Plane> >> planes_;

};

}
#endif /* CREATE_POINTS_HPP_ */
