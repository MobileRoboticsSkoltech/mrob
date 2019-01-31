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



namespace mrob{


class Plane{
  public:
    Plane(SE3);
    ~Plane();


  protected:
    // Overparametrized plane, as a transformation
    SE3 plane_, estimatedPlane_;
    // for now a vector, is it necessary a queue to remove data?
    // Local copy of points
    std::vector<MatX> data_;

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
    std::vector<SE3> get_transformations();//if solved

  protected:
    std::vector<Plane> planes_;//we should include here references
    std::vector<SE3> transformations_;
    // flag for detecting when is has been solved
    uint_t isSolved_;
    uint_t time_;
};



}// namespace
#endif /* PLANEREGISTRATION_HPP_ */
