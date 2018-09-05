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

#include "skmr/matrix_base.hpp"
#include "skmr/SE3.hpp"

/**
 * This header provides the common structure for the Point Cloud Alignment,
 * that is, a common abstract class structure, common data variables and methods
 *
 * In general we will assume that point are
 */




namespace skmr{

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

/**
 * PC_t point cloud class provides an easy way to store data and auto remove its content
 * via pointer definitions
 *
 * TODO : to be deperrcated?? who uses this?
 */
class PC_t{
  public:
    PC_t(int N);
    ~PC_t();
    void add_point(Point3_t);
    void print();
  private:
    std::vector<Point3_t> X;
};

class BaseTransf{
  public:
    BaseTransf(const std::shared_ptr<MatX> &X, const std::shared_ptr<MatX> &Y);//TODO input an array of data, and then a MAP to Eigen
    virtual ~BaseTransf();
    virtual int solve(void) = 0;
    SE3 getT(){return T;};
  protected:
    // TODO will this be PCL?? I need to separe projects then
    //std::shared_ptr<PC_t> X;
    std::shared_ptr<MatX> X;
    std::shared_ptr<MatX> Y;
    SE3 T;
    uint_t N_;
};


}

#endif /* SRC_BASE_T_HPP_ */
