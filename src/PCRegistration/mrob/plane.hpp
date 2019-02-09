/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * plane.hpp
 *
 *  Created on: Feb 1, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef PLANE_HPP_
#define PLANE_HPP_


#include <vector>
#include "mrob/SE3.hpp"
#include <Eigen/StdVector>



namespace mrob{

/**
 * class Plane stores a vector of point clouds from different observations
 * and  provides the gradients with respect to those poses.
 *
 */
class Plane{
  public:
    Plane(uint_t timeLength);
    ~Plane();
    void reserve(uint_t d, uint_t t);
    void set_plane(SE3 &);
    SE3 get_plane(void);
    /**
     * This function is intended to add a point that belongs to the plane at time t
     * Internally, we will save a copy of them
     */
    void push_back_point(Mat31 &point, uint_t t);
    std::vector<Mat31>& get_points(uint_t t);
    void clear_points();

    void print() const;


  protected:
    // max index of time, or trajectory length
    uint_t timeLength_;
    //std::vector<uint_t> timeIndex_;// not used now, but it would be useful for non-consecutive obs

    // Overparametrized plane, as a transformation in SE3
    SE3 plane_;

    std::vector< std::vector<Mat31> > allPlanePoints_;

};
}


#endif /* PLANE_HPP_ */
