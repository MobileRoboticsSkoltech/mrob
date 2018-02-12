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

#include "SE3.hpp"


namespace align_mth{

/**
 * Class samples a random configuration on SE3 using a
 * uniform distribution U(-R_range, R_range)
 * or any implemented distribution
 */
class Csample_uniform_SE3{
  public:
    Csample_uniform_SE3(double R_range, double t_range);
    Csample_uniform_SE3(double R_min, double R_max, double t_min, double t_max);
    ~Csample_uniform_SE3();
    lie::SE3 sample();
  protected:
    std::default_random_engine generator_;
    std::uniform_real_distribution<double> R_uniform_;
    std::uniform_real_distribution<double> t_uniform_;
};




/**
 * Class generating a pair of pointclouds given some specifications.
 * It also produces normals and planes, as if we had pre-processed points
 */
class Cpair_PC{
public:
    Cpair_PC();
    ~Cpair_PC();
    // samples a new transformation, and random
    void sample();
    // Point cloud data generated
    Eigen::MatrixXd X, Y;
    lie::SE3 T_ground_truth;

    // generation parameters
    int N;
    double R_range;
    double t_range;
    double std_y;
    double lamda_outlier;
    double R_normal;
    double normal_noise;
    double normal_noise_per_point;
    double epsilon;
    double cov_with_dist;
    double range;
    int sample_on_plane;
    int count_points_on_plane;
    int max_number_point_on_plane;
    double weight_factors;
protected:
};

}
#endif /* CREATE_POINTS_HPP_ */
