/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * planeRegistration.hpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef PLANEREGISTRATION_HPP_
#define PLANEREGISTRATION_HPP_


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
    void set_plane(SE3 &);
    SE3 get_plane(void);


  protected:
    // index of time,
    uint_t timeLength_;
    std::vector<uint_t> timeIndex_;

    // Overparametrized plane, as a transformation in SE3
    SE3 plane_;

    // Local copy of points
    // XXX for now a vector, is it necessary a queue to remove data?
    std::vector<MatX> points_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * class PlaneRegistration introduced a class for the alignment of
 * planes.
 */
class PlaneRegistration{
  public:
    PlaneRegistration();
    ~PlaneRegistration();

    int solve();
    std::vector<SE3>& get_transformations();//if solved

  protected:
    std::vector<Plane> planes_;//we should include here references
    std::vector<SE3> transformations_;
    // flag for detecting when is has been solved
    uint_t isSolved_;
    uint_t time_;
};



}// namespace
#endif /* PLANEREGISTRATION_HPP_ */
