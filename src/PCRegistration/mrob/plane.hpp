/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
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
#include <memory>
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
    Mat41 get_plane(void) {return planeEstimation_;};
    /**
     * This function is intended to add a point that belongs to the plane at time t
     * Internally, we will save a copy of them
     */
    void push_back_point(Mat31 &point, uint_t t);
    std::vector<Mat31>& get_points(uint_t t);
    uint_t get_number_points(uint_t t) {return allPlanePoints_[t].size();};
    uint_t get_total_number_points() {return numberPoints_;};
    void clear_points();
    void set_trajectory(const std::shared_ptr<const std::vector<SE3>> &trajectory) {trajectory_ = trajectory;};

    /**
     * Estimates the plane parameters: v = [n', d]'_{4x1}, where v is unitary, (due to the SVD solution)
     * although for standard plane estimation we could enforce unitary on the normal vector n.
     */
    double estimate_plane();
    /**
     * Estimates Incremetally the plane parameters: v = [n', d]'_{4x1}, where v is unitary, (due to the SVD solution)
     * although for standard plane estimation we could enforce unitary on the normal vector n.
     *
     * The difference with the previous estimate_plane() is that we update the matrix Q for the give time
     * stamp and recalculate the solution, on constant time O(1)
     */
    double estimate_plane_incrementally(uint_t t);
    /**
     * get error: returns the error as the min eigenvalue
     */
    double get_error() const {return lambda_;};
    /**
     * get error incremental: returns the error as the min eigenvalue only updating the
     * value of Q_t, at time step t. Nothing insie gets updated
     */
    double get_error_incremental(uint_t t) const;

    /**
     *  calculates the matrix S = sum(p*p'), where p = [x,y,z,1]
     * for all planes, as an aggregation of the outer product of all
     * homogeneous points
     * If reset = true, clears all information and recalculates S
     * If reset = false (default) only calculates S if there is no calculation yet
     */
    void calculate_all_matrices_S(bool reset=false);
    /**
     * get mean point calculates the mean of the pointcloud observed at time t,
     * given that S = sum p * p' =  sum ([x2 xy xz x
     *                                    yx y2 yz y
     *                                    zx zy z2 z
     *                                    x   y  z 1]
     * ser we just calcualte S and return
     */
    Mat31 get_mean_point(uint_t t);
    /**
     *  calculates the matrix Qi = 1^T_i * Si * (1^T_i)^transp
     *  for all planes. Since this is an iterative process on T's,
     *  we separate the calculation of the S matrix,
     *  and the Q matrix which rotates S
     */
    void calculate_all_matrices_Q();
    /**
     * calculate Jacobian at time t, returns the Jacobian over the SVD od Q  = U D V'
     * as d lamda = v' * dQ * v
     *
     */
    Mat61 calculate_jacobian(uint_t t);


    void reset();
    void print() const;


  protected:
    // max index of time, or trajectory length
    uint_t timeLength_;
    //std::vector<uint_t> timeIndex_;// not used now, but it would be useful for non-consecutive obs

    // Overparametrized plane, as a transformation in SE3 TODO needed?
    //SE3 plane_;
    Mat41 planeEstimation_;
    double lambda_;
    bool isPlaneEstimated_;// XXX used or deprecated?

    // subset of pointcloud for the given plane
    std::vector< std::vector<Mat31> > allPlanePoints_;
    uint_t numberPoints_;

    // Transformations. We shared it across all planes that need it.
    std::shared_ptr<const std::vector<SE3>> trajectory_;

    // gradient calculation Q
    std::vector<Mat4> matrixS_, matrixQ_;
    Mat4 accumulatedQ_;//Q matrix of accumulated values for the incremental update of the error.

  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}


#endif /* PLANE_HPP_ */
