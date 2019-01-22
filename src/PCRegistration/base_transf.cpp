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

//TODO Ref is copying things, what is woroing!!!
BaseTransf::BaseTransf(const Eigen::Ref<const MatX> X, const Eigen::Ref<const MatX> Y):
        X_(X), Y_(Y)
{
    assert(X.rows() == 3  && "BaseTransf::BaseTransf: Incorrect sizing, we expect 3xN");
    assert(X.cols() >= 3  && "BaseTransf::BaseTransf: Incorrect sizing, we expect at least 3 correspondences (not aligned)");
    assert(Y.cols() == X.cols()  && "BaseTransf::BaseTransf: Same number of correspondences");
    N_ = X.cols();
    //std::cout << "X data: \n" << X_ << std::endl;
    //std::cout << "Y data: \n" << Y_ << std::endl;
}

BaseTransf::~BaseTransf()
{
}
