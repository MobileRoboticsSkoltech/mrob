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
 * SE3.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include "mrob/SE3.hpp"
#include <cmath>
#include <iostream>
#include <memory>


using namespace mrob;

SE3::SE3(const Mat4 &T) :
        T_(T)
{
}

SE3::SE3(const Mat61 &xi) : T_(Mat4::Identity())
{
    //std::cout << "SE3 MAT31" << std::endl;
    this->exp(hat6(xi));
}

SE3::SE3(const SE3 &T): T_(T.T())
{
}
SE3::SE3(const SO3 &R, const Mat31 t)
{
    T_  << R.R(), t,
           0,0,0,1;
}

SE3::SE3(const Mat3 &R, const Mat31 t)
{
    T_  << R, t,
           0,0,0,1;
}
template<typename OtherDerived>
SE3::SE3(const Eigen::MatrixBase<OtherDerived>& rhs)  :
    T_(rhs)
{    //std::cout << "SE3 MAT4" << std::endl;
}


SE3& SE3::operator=(const SE3& rhs)
{
    // check for self assignment TODO
    if (this == &rhs)
        return *this;
    T_ = rhs.T();
    return *this;
}

SE3 SE3::operator*(const SE3& rhs) const
{
    Mat4 res = T_ * rhs.T();
    return SE3(res);
}


SE3 SE3::mul(const SE3& rhs) const
{
    return (*this) * rhs;
}

void SE3::update_lhs(const Mat61 &dxi)
{
    SE3 dT(dxi);
    T_ = dT.T() * T_;
}
void SE3::update_rhs(const Mat61 &dxi)
{
    SE3 dT(dxi);
    T_ = T_ * dT.T();
}

Mat61 mrob::vee6(const Mat4 &xi_hat)
{
    Mat61 xi;
    xi << -xi_hat(1,2), xi_hat(0,2), -xi_hat(0,1),
           xi_hat(0,3), xi_hat(1,3), xi_hat(2,3);
    return xi;
}

Mat4 mrob::hat6(const Mat61 &xi)
{
    Mat4 xi_hat;
    xi_hat  <<    0.0, -xi(2),  xi(1), xi(3),
                xi(2),    0.0, -xi(0), xi(4),
               -xi(1),  xi(0),    0.0, xi(5),
                    0,      0,      0,    0;
    return xi_hat;
}

void SE3::exp(const Mat4 &xi_hat)
{
    // Calculating xi = [w, v]
    Mat61 xi = vee6(xi_hat);
    Mat31 w = xi.head<3>();
    Mat31 v = xi.tail<3>();
    SO3 rotation(w);
    Mat3 w_hat = xi_hat.topLeftCorner<3,3>();

    // Calculate the closed form of V
    // V = I + c2*(w^) + c3*(w^)^2   ,
    // where o = norm(w), c2 = (1 - cos(o))/o^2, c3 = (o- sin(o) / o^3
    Mat3 V = Mat3::Identity();
    double o = w.norm();
    double o2 = w.squaredNorm();
    // If rotation is not zero
    matData_t c2, c3;
    if ( o > 1e-3){ // c2 and c3 become numerically imprecise for o < 1-5, so we choose a conservative threshold 1e-3
        c2 = (1 - std::cos(o))/o2;
        c3 = (o - std::sin(o))/o2/o;
    }
    else
    {
        // second order Taylor (first order is zero since this is an even function)
        c2 = 0.5 - o2/24;
        // Second order Taylor
        c3 = 1.0/6.0 - o2/120;
    }
    V += c2*w_hat + c3*w_hat*w_hat;

    // Calculate the translation component t = Vv
    Mat31 t = V*v;

    // compose the rigid body motion matrix T = [R, t]
    //this->topLeftCorner<3,3>() = R;
    //this->topRightCorner<3,1>() = t;
    T_  << rotation.R(), t,
           0,0,0,1;
}

Mat4 SE3::ln(void) const
{
    SO3 rotation(this->R());
    // Logarithmic mapping of the rotations
    double o; // This is absolute value of angle
    Mat3 w_hat = rotation.ln(&o);

    // calculate v = V^1 t
    // V^-1 = I - 0.5w^ + k1 (w^)^2
    // k1 = 1/o^2 * (1 - c1/(2c2) ) ,    c1 =sin(o)/o and c2 = (1 - cos(o))/o^2 from so3_exp
    Mat3 Vinv = Mat3::Identity();
    double k1;
    // 5e-3 bound provided on numerical_test.cpp, smaller than this k1 becomes degradated
    if (o > 5e-3)
    {
        double c1 = std::sin(o); //sin(o)/o, we remove the o in both coeficients
        double c2 = (1 - std::cos(o))/o; // (1 - std::cos(o))/o/o
        k1 = 1/o/o*(1 - 0.5*c1/c2);
    }
    //Taylor expansion for small o.
    else
    {
        // f(o) = 1/12 + 1/2*f''*o^2
        // f'' = 1/360
        k1 = 1.0/12 + o*o/720;
    }
    Vinv += -0.5*w_hat + k1* w_hat*w_hat;

    // v = V^-1 t
    Mat31 v = Vinv * T_.topRightCorner<3,1>();

    // create a vector containing the components
    Mat4 xi_hat = Mat4::Zero();
    xi_hat << w_hat, v,
              0,0,0,0;
    return xi_hat;
}

Mat61 SE3::ln_vee() const
{
    Mat4 xi_hat = this->ln();
    return vee6(xi_hat);
}

Mat31 SE3::transform(const Mat31 & p) const
{
    return R()*p + t();
}


MatX SE3::transform_array(const MatX &P) const
{
    assert(P.cols() == 3 && "SE3::transformArray: incorrect data structure, it is required an Nx3 input");
    uint_t N = P.rows();
    MatX res(N,3);
    for (uint_t i = 0; i < N; ++i)
        res.row(i) << this->transform(P.row(i)).transpose();
    return res;
}


SE3 SE3::inv(void) const
{
    Mat4 inv;
    Mat3 R = this->R();
    R.transposeInPlace();
    inv << R, -R * this->t(),
           0,0,0,1;
    return SE3(inv);

}

Mat6 SE3::adj() const
{
    Mat6 res(Mat6::Zero());
    Mat3 tx = hat3( this->t() );
    res.topLeftCorner<3,3>() << R();
    res.bottomRightCorner<3,3>() << R();
    res.bottomLeftCorner<3,3>() << tx*R();
    return res;
}

//Mat4 SE3::T() const
const Eigen::Ref<const Mat4> SE3::T() const
{
    return T_;
}

Mat4& SE3::ref2T()
{
    return T_;
}

Mat3 SE3::R() const
{
    return T_.topLeftCorner<3,3>();
}

Mat31 SE3::t() const
{
    return T_.topRightCorner<3,1>();
}

double SE3::distance(const SE3 &rhs) const
{
    return (*this * rhs.inv()).ln_vee().norm();
}

double SE3::distance_rotation(const SE3 &rhs) const
{
    Mat3 dR = this->R() * rhs.R().transpose();
    return SO3( dR ).ln_vee().norm();
}

double SE3::distance_trans(const SE3 &rhs) const
{
    return (this->t() - rhs.t()).norm();
}
void SE3::print(void) const
{
    std::cout << T_ << std::endl;
}


void SE3::print_lie(void) const
{

    std::cout << this->ln_vee() << std::endl;
}

void SE3::regenerate()
{
    Mat4 xi_hat = this->ln();
    this->exp(xi_hat);
}

bool mrob::isSE3(Mat4 T)
{
    if (!isSO3(T.topLeftCorner<3,3>()) )
        return false;
    Mat<1,4> one;
    one << 0,0,0,1;
    if ( (T.row(3)-one).sum() > 1e-6 )
        return false;
    return true;
}
