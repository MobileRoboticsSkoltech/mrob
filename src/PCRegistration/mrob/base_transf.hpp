/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * base_transformation.hpp
 *
 *  Created on: Jan 31, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef SRC_BASE_TRANSF_HPP_
#define SRC_BASE_TRANSF_HPP_


#include <vector>
#include <memory>
#include <iostream>

#include "mrob/matrix_base.hpp"
#include <Eigen/LU> // for inverse and determinant
#include "mrob/SE3.hpp"


/**
 * This header provides the common structure for the Point Cloud Alignment,
 * that is, a common abstract class structure, common data variables and methods
 *
 * In general we will assume that points and PointCloud is from the PCL library?
 */




namespace mrob{

//TODO is this used? To be deprecated
class Point3_t{
public:
    Point3_t(void);
    Point3_t(double x, double y, double z);
    Point3_t(double d[3]);
    ~Point3_t();
    void print(void);
    double x;
    double y;
    double z;
};


class BaseTransf{
  public:
    //BaseTransf(const MatX &X, const MatX &Y);
    BaseTransf(const Eigen::Ref<const MatX> X, const Eigen::Ref<const MatX> Y);
    virtual ~BaseTransf();
    virtual int solve(void) = 0;
    SE3 getT(){return T_;};
  protected:
    // Data is passed through an Eigen::Ref object, allocated outside this class.
    // it is the designer responsability to allocate these data.
    const MatX X_, Y_;
    SE3 T_;
    uint_t N_;

};


}

#endif /* SRC_BASE_T_HPP_ */
