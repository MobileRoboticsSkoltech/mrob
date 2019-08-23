/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * SO3.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include "mrob/SO3.hpp"
#include <cmath>
#include <iostream>
#include <Eigen/LU> // for determinant
#include <Eigen/Geometry> // for quaternions and rotations

// TODO remove
#include <iomanip>

using namespace mrob;

SO3::SO3(const Mat3 &R) :
        R_(R)
{
}

SO3::SO3(const Mat31 &w) :
        R_(Mat3::Identity())
{
    //std::cout << "SO3 with Mat31" << std::endl;
    this->exp(hat3(w));
}

SO3::SO3(const SO3 &R) :
        R_(R.R())
{
}

template<typename OtherDerived>
SO3::SO3(const Eigen::MatrixBase<OtherDerived>& rhs)  :
    R_(rhs)
{    //std::cout << "SE3 MAT4" << std::endl;
}

SO3& SO3::operator=(const SO3 &rhs)
{
    //std::cout << "SO3 operator equal" << std::endl;
    // check for self assignment
    if (this == &rhs)
        return *this;
    R_ = rhs.R();
    return *this;
}

SO3 SO3::operator*(const SO3& rhs) const
{
    Mat3 res = R_ * rhs.R();
    return SO3(res);
}

SO3 SO3::mul(const SO3& rhs) const
{
    return (*this) * rhs;
}

void SO3::update_lhs(const Mat31 &dw)
{
    SO3 dR(dw);
    R_ = dR.R() * R_;
}

void SO3::update_rhs(const Mat31 &dw)
{
    SO3 dR(dw);
    R_ = R_ * dR.R();
}

Mat31 mrob::vee3(const Mat3 &w_hat)
{
    Mat31 w;
    w << -w_hat(1,2), w_hat(0,2), -w_hat(0,1);
    return w;
}

Mat3 mrob::hat3(const Mat31 &w)
{
    Mat3 w_hat;
    w_hat <<     0.0, -w(2),  w(1),
                w(2),   0.0, -w(0),
               -w(1),  w(0),   0.0;
    return w_hat;
}


void SO3::exp(const Mat3 &w_hat)
{
    Mat31 w = vee3(w_hat);
    double o = w.norm();
    double c1,c2;
    // TODO justify this threshold
    if ( o < 1e-5){
        // sin(o)/0 -> 1. Approximate this with Taylor
        //R_ << Mat3::Identity() + w_hat;
        // sin(o)/o = 1 - x^2/3! + x^4/5! + O(x^6)
        c1 =  1 - o*o/6.0;
        // (1-cos(o))/o^2 = 0.5 + x^2/12 + ...
        c2 = 0.5 - o*o/12.0;
        //std::cout << "here\n";
    }
    else
    {
        c1 = std::sin(o)/o;
        c2 = (1 - std::cos(o))/o/o;
    }
    R_ << Mat3::Identity() + c1 * w_hat + c2 * w_hat *w_hat;
}

Mat3 SO3::ln(double *ro) const
{
    // Logarithmic mapping of the rotations
    Mat3 res;
    double tr = (R_.trace()-1)*0.5;
    double o = std::acos(tr); //[0,pi]
    double lnTol = 1e-9;
    if (tr  < 1.0 - lnTol && tr > -1.0 + lnTol )
    {
        // Usual case, tr \in (-1,1) and theta \in (-pi,pi)
        res << 0.5 * o / std::sin(o) * ( R_ - R_.transpose());
    }
    else if (tr >= 1.0 - lnTol )
    {
        // Special case tr =1  and theta -> 0
        // We will evaluate 3 cases: 1) incorrect o 2) incorrect x.sin 3) normal
        double d1;
        if (o != o) // is Nan
        {
            d1 = 0.0;
        }
        else if( o < lnTol)
        {
            // o can be exactly 0 (=acos) which is why we need this statement, although it might be redundant
            // and a Taylor expansion for such a small o, if not, Nans TODO is taylor correct?
            d1 = 0.5 + o*o/12;
        }
        else
        {
            d1 = 0.5 * o / std::sin(o);
        }
        res << d1 * ( R_ - R_.transpose());
    }
    else
    {
        // Special case tr = -1  and theta = +- pi or multiples
        //std::cout << "pi pi o = " << o << std::endl;
        if ( o != o)
        {
            o = M_PI;// exact case for theta
            std::cout << "here starts the error\n";
        }
        else
        {
            // Taylor around the cos
            std::cout << o << std::endl;
            o = M_PI;
        }
        // First order Taylor to approximate acos at pi
        //o = M_PI - 1.0/std::sqrt(1.0-tr*tr) * (1 - std::fabs(tr));

        // As we approach pi, the Taylor expansion becomes:
        // R = I + 0 + (2/pi^2)W^2, which makes it symmetric R = Rt and W = hat(w)
        // so we can consider the first order term negligible.
        //
        // From here, we know that W^2 = ww^t - theta^2I, (you can span W^2 to see this)
        // which leaves R = I + 2/pi2 (wwt - pi2 I)
        // R+I = 2/pi2 wwt
        // wwt = pi2 / 2 (R+I)
        // so we find the maximum row and apply that formula
        // knowing that norm(w) = pi
        Mat31 w;
        if( R_(0,0) > R_(1,1) && R_(0,0) > R_(2,2) )
        {
            // For stability, we average the two elements since it must be symetric
            w << R_(0,0) + 1.0,
                 0.5 * ( R_(0,1) + R_(1,0)),
                 0.5 * ( R_(0,2) + R_(2,0));
        }
        else if( R_(1,1) > R_(0,0) && R_(1,1) > R_(2,2) )
        {
            w << 0.5 * ( R_(1,0) + R_(0,1)),
                 R_(1,1) + 1.0,
                 0.5 * ( R_(1,2) + R_(2,1));
        }
        else
        {
            w << 0.5 * ( R_(2,0) + R_(0,2)),
                 0.5 * ( R_(2,1) + R_(1,2)),
                 R_(2,2) + 1.0;
        }
        // normalize the vector w, such that norm(w) = pi
        double length = w.norm();
        if (length > 0.0)
        {
            w *= o / length;
        }
        else
        {
            w << 0.0, 0.0, 0.0;
        }
        res = hat3(w);
    }
    if (ro != nullptr) *ro = o;
    return res;
}

Mat31 SO3::ln_vee() const
{
    Mat3 w_hat = this->ln();
    return vee3(w_hat);
}

SO3 SO3::inv(void) const
{
    return SO3(R_.transpose());
}

Mat3 SO3::adj() const
{
    return R_;
}

Mat3 SO3::R() const
{
    return R_;
}

Mat3& SO3::ref2R()
{
    return R_;
}

double SO3::distance(const SO3 &rhs) const
{
    return (*this * rhs.inv()).ln_vee().norm();
}

void SO3::print(void) const
{
    std::cout << R_ << std::endl;
}


void SO3::print_lie(void) const
{

    Mat31 w =  this->ln_vee();
    std::cout << w << std::endl;
}

bool mrob::isSO3(Mat3 R)
{
    matData_t det = R.determinant();
    if (det  < 0)
        return false;
    if ( fabs(det - 1.0) > 1e-6)
        return false;
    return true;

}


Mat3 mrob::quat_to_so3(const Eigen::Ref<const Mat41> v)
{

    Eigen::Quaternion<matData_t> q(v);
    //std::cout << "Initial vector : " << v << ", transformed quaternion" << q.vec() << "\n and w = \n" << q.toRotationMatrix() << std::endl;
    return q.normalized().toRotationMatrix();
}

// quaternion q = [qx, qy, qz, qw](Eigen convention)
Mat41 mrob::so3_to_quat(const Eigen::Ref<const Mat3> R)
{
    Eigen::Quaternion<matData_t> q(R);
    Mat41 res;
    res << q.x(), q.y(), q.z(), q.w();
    return res;
}

Mat3 mrob::rpy_to_so3(const Eigen::Ref<const Mat31> v)
{
    Mat3 R;
    R = Eigen::AngleAxisd(v(0), Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(v(1), Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(v(2), Eigen::Vector3d::UnitZ());
    return R;
}
