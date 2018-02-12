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


#include "create_points.hpp"
#include <random>
#include <chrono>
#include <assert.h>
#include <iostream>
#include <cmath>

using namespace align_mth;

Csample_uniform_SE3::Csample_uniform_SE3(double R_range, double t_range):
    R_uniform_(-R_range, R_range), t_uniform_(-t_range, t_range)
{
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.seed(seed);
}

Csample_uniform_SE3::Csample_uniform_SE3(double R_min, double R_max, double t_min, double t_max):
    R_uniform_(R_min, R_max), t_uniform_(t_min, t_max)
{
    assert(R_min <= R_max && "\nCsample_uniform_SE3::Csample_uniform_SE3 incorrect R bounds");
    assert(t_min <= t_max && "\nCsample_uniform_SE3::Csample_uniform_SE3 incorrect t bounds");
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.seed(seed);
}


Csample_uniform_SE3::~Csample_uniform_SE3()
{
}

lie::SE3 Csample_uniform_SE3::sample()
{
    // xi = [w , v]^T
    Mat61 xi;
    xi <<   R_uniform_(generator_),
            R_uniform_(generator_),
            R_uniform_(generator_),
            t_uniform_(generator_),
            t_uniform_(generator_),
            t_uniform_(generator_);
    return lie::SE3(xi);
}

Cpair_PC::Cpair_PC():
        N(10),
        R_range(M_PI),
        t_range(10.0),
        std_y(1.5),
        lamda_outlier(0.0),
        R_normal(M_PI/10),
        normal_noise(0.1),
        normal_noise_per_point(0.0), //deprecated?
        epsilon(1e-3),
        cov_with_dist(0.1),
        range(30),
        sample_on_plane(1), // 0 for sampling over the point, 1 for sampling over the plane
        count_points_on_plane(0), // 0 for not counting the number of planes, 1 for "counting" point
        max_number_point_on_plane(100),
        weight_factors(1.0)
{

}

Cpair_PC::~Cpair_PC()
{

}
