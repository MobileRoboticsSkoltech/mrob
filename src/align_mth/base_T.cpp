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

#include "base_T.hpp"

using namespace align_mth;

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
base_T::base_T(const std::shared_ptr<Eigen::MatrixXd>  &X_, const std::shared_ptr<Eigen::MatrixXd> &Y_):
    X(X_), Y(Y_)
{
}

base_T::~base_T()
{
}
