/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * base_transf.cpp
 *
 *  Created on: Feb 1, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "mrob/base_transf.hpp"

using namespace mrob;

Point3_t::Point3_t(void):
        x(0.0), y(0.0), z(0.0)
{ }

Point3_t::Point3_t(double x_, double y_, double z_) :
        x(x_), y(y_), z(z_)
{ }

Point3_t::Point3_t(double d[3]) :
        x(d[0]), y(d[1]), z(d[2])
{ }

Point3_t::~Point3_t()
{ }

void Point3_t::print(void)
{
    std::cout << "[" << x << ", " << y << ", "<< z << "]" << std::endl;
}

BaseTransf::BaseTransf(const std::shared_ptr<MatX> &X_, const std::shared_ptr<MatX> &Y_)
{
    assert(X_->cols() >= X_->rows()  && "base_T::base_T: Incorrect sizing, we expect 3xN");
    assert(X_->rows() == 3  && "base_T::base_T: Incorrect sizing, we expect 3xN");
    assert(Y_->cols() == X_->cols()  && "base_T::base_T: Registration PC not equal");
    N_ = X_->cols();// we expect column matrices 3xN

    // For PCL, to be removed??
    //assert(X->points.size() != 0 );
    //assert(Y->points.size() != 0 );
}

BaseTransf::~BaseTransf()
{
}
