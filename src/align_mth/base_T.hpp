/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * base_T.hpp
 *
 *  Created on: Jan 31, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef SRC_BASE_T_HPP_
#define SRC_BASE_T_HPP_


#include <vector>
#include <memory>
#include <iostream>
#include <Eigen/Dense>
#include "SE3.hpp"

/**
 * This header provides the common structure for the Point Cloud Alignment,
 * that is, a common abstract class structure, common data variables and methods
 *
 * In general we will assume that point are
 */




namespace align_mth{

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

class base_T{
  public:
    base_T(const std::shared_ptr<Eigen::MatrixXd> &X);
    virtual ~base_T();
    virtual int solve(void) = 0;
  protected:
    //std::shared_ptr<PC_t> X;
    std::shared_ptr<Eigen::MatrixXd> X;
    std::shared_ptr<Eigen::MatrixXd> Y;
    lie::SE3 T;
};


}

#endif /* SRC_BASE_T_HPP_ */
