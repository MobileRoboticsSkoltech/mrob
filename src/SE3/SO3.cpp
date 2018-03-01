/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * SO3.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include "SO3.hpp"
#include <cmath>
#include <iostream>


using namespace lie;

SO3::SO3(const Mat31 &w) : Mat3(Mat3::Identity())
{
    //std::cout << "SO3 with Mat31" << std::endl;
    this->exp(this->hat(w));
}
template<typename OtherDerived>
SO3::SO3(const Eigen::MatrixBase<OtherDerived>& other)  :
Mat3(other)
{
    //std::cout << "SO3 MAT3" << std::endl;
}

template<typename OtherDerived>
SO3& SO3::operator=(const Eigen::MatrixBase <OtherDerived>& other)
{
    //std::cout << "SO3 operator equal" << std::endl;
    this->Mat3::operator=(other);
    return *this;
}

void SO3::update(const Mat31 &dw)
{
    SO3 dR(dw);
    *this = dR * (*this);
}

Mat31 SO3::vee(const Mat3 &w_hat) const
{
    Mat31 w;
    w << -w_hat(1,2), w_hat(0,2), -w_hat(0,1);
    return w;
}

Mat3 SO3::hat(const Mat31 &w) const
{
    Mat3 w_hat;
    w_hat <<     0.0, -w(2),  w(1),
                w(2),   0.0, -w(0),
               -w(1),  w(0),   0.0;
    return w_hat;
}


void SO3::exp(const Mat3 &w_hat)
{
    Mat31 w = this->vee(w_hat);
    double o = w.norm();
    if ( o < 1e-12){
        *this << Mat3::Identity();
        return;
    }
    double c1 = std::sin(o)/o;
    double c2 = (1 - std::cos(o))/o/o;
    *this << Mat3::Identity() + c1 * w_hat + c2 * w_hat *w_hat;
}

Mat3 SO3::ln(double *ro) const
{
    // Logarithmic mapping of the rotations
    double o = std::fabs(std::acos((this->trace()-1)*0.5));
    if (ro != nullptr) *ro = o;
    if ( o > 1e-9)
    {
        return 0.5 * o / std::sin(o) * ( (*this) - this->transpose());
    }
    else
    {
        return Mat3::Zero();
    }
}

Mat31 SO3::ln_vee() const
{
    Mat3 w_hat = this->ln();
    return this->vee(w_hat);
}

SO3 SO3::inv(void) const
{
    return this->transpose();
}

void SO3::print(void) const
{
    std::cout << *this << std::endl;
}


void SO3::print_lie(void) const
{

    Mat31 w =  this->ln_vee();
    std::cout << w << std::endl;
}
