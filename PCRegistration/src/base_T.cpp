/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * base_T.cpp
 *
 *  Created on: Feb 1, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#include "skmr/base_T.hpp"

using namespace skmr;

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
PC_t::PC_t(int N)
{
    X.reserve(N);
}
PC_t::~PC_t()
{
    X.clear();
}

void PC_t::print()
{
    for (Point3_t &element : X)
    {
        element.print();
    }
}

void PC_t::add_point(Point3_t p)
{
    X.push_back(p);
}
Base_T::Base_T(const std::shared_ptr<MatX>  &X_, const std::shared_ptr<MatX> &Y_):
    X(X_), Y(Y_)
{
    assert(X->cols() >= X->rows()  && "base_T::base_T: Incorrect sizing, we expect 3xN");
    assert(X->rows() == 3  && "base_T::base_T: Incorrect sizing, we expect 3xN");
    assert(Y->cols() == X->cols()  && "base_T::base_T: Registration PC not equal");
    N_ = X->cols();// we expect column matrices 3xN
}

Base_T::~Base_T()
{
}
