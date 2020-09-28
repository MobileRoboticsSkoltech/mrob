/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
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
    // See numerical_test.cpp to justify this thershold
    if ( o < 1e-5){
        // sin(o)/o = 1 - x^2/3! + x^4/5! + O(x^6)
        c1 =  1 - o*o/6.0;
        // (1-cos(o))/o^2 = 0.5 + 1/2*f''*x^2/ + ... , where f'' = 1/12
        c2 = 0.5 - o*o/24.0;
    }
    else
    {
        // Standard case with the well-known Rodriguez formula
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
    double o = std::acos(tr); //image in [0,pi]
    // We choose a tolerance of this small value since the solution involves R being almost symetric
    if ( tr > -1.0 + 1e0)
    {
        // We will evaluate 3 cases:
        // 1) o (angle) is NaN because trace was exactly 1 plus a small round off, so acos is not defined
        double d1;
        // Special case tr =1  and theta -> 0
        if ( std::isnan(o) )
        {
            d1 = 0.0;
            o = 0.0;
        }
        // 2) incorrect x/sin when approaching 0. The other problematic point o = pi is handled below
        // We choose this value since Taylor improves over the numerical result of the program (see numericap_test.cpp)
        else if( o < 1e-5)
        {
            // Taylor expansion around 0
            d1 = 0.5 + o*o/12;
        }
        // 3) normal case
        else
        {
            d1 = 0.5 * o / std::sin(o);
        }
        res << d1 * ( R_ - R_.transpose());
    }
    else
    {
        // Special case tr = -1  so theta = + pi or multiples
        // Again, we handle nan's assuming they express exact +-pi plus a numerical error
        // The second condition stand for the error in acos. Very close to pi it is better (in error) to assume
        // a rotation of exactly pi
        if ( std::isnan(o) || M_PI - o < 6e-8 )
        {
            o = M_PI;// exact case for theta
        }
        // In this case we assume theta is well calculated, but given the almost symetry conditions
        // on the expansion that allow to solve the problem this way.
        else
        {
            // The result of acos is good enough
            //std::cout << std::setprecision(20) << M_PI - o << std::endl;
        }
        // As we approach pi, the exponent(theta) becomes:
        // R = I + 0 + (1-cos)/o^2)W^2, which evaluated at +pi = 2/pi^2
        // This rotation is almost symmetric R = Rt and W = hat(w)
        // We can consider the first order term sin/o negligible.
        //
        // From here, we know that W^2 = ww^t - theta^2I, (you can span W^2 to see this)
        // which leaves R = I + 2/pi2 (wwt - pi2 I)
        // R+I = 2/pi2 wwt
        // wwt = pi2 / 2 (R+I)
        // so we find the maximum row and apply that formula
        // knowing that norm(w) = pi (theta)
        Mat31 w;
        double d = std::cos(o);

        if( R_(0,0) > R_(1,1) && R_(0,0) > R_(2,2) )
        {
            // For stability, we average the two elements since it is almost symetric
            w << R_(0,0) - d,
                 0.5 * ( R_(0,1) + R_(1,0)),
                 0.5 * ( R_(0,2) + R_(2,0));
        }
        else if( R_(1,1) > R_(0,0) && R_(1,1) > R_(2,2) )
        {
            w << 0.5 * ( R_(1,0) + R_(0,1)),
                 R_(1,1) - d,
                 0.5 * ( R_(1,2) + R_(2,1));
        }
        else
        {
            w << 0.5 * ( R_(2,0) + R_(0,2)),
                 0.5 * ( R_(2,1) + R_(1,2)),
                 R_(2,2) - d;
        }
        // normalize the vector w, such that norm(w) = theta
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

        // NOTE: this does not work for very very small theta, which is expectedsince at pi the axis is undefined
        // The problem of this approach is that we loose the sign of rotation, so we try to estimate it
        // by comparing R with the 1st order of Exp(w) and Exp(-w).
        if( (Mat3::Identity() + res - R_ ).norm() >  (Mat3::Identity() - res - R_).norm() )
        {
            res *= -1.0;
            // Note that o is the absolute value of the angle of rotation (don't flip)
        }
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
