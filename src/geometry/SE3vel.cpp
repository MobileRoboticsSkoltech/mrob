#include "mrob/SE3vel.hpp"

using namespace mrob;

SE3vel::SE3vel(const Mat5 &T) : T_(T) {}

SE3vel::SE3vel(const SE3vel &T) : T_(T.T()){}

SE3vel::SE3vel(const Mat91 &xi)
{
    this->Exp(xi);
}
 
Mat31 SE3vel::t() const
{
    return T_.topRightCorner<3,1>();
}

Mat31 SE3vel::v() const
{
    return T_.block<3,1>(0,3);
}

Mat3 SE3vel::R() const
{
    return T_.topLeftCorner<3,3>();
}

Mat5 SE3vel::T(void) const
{
    return this->T_;
}

SE3vel SE3vel::inv(void) const
{
    Mat5 inv(Mat5::Zero());
    Mat3 R = this->R();
    R.transposeInPlace();

    inv << R, -R*this->v(), -R*this->t(),
            0,0,0,1,0,
            0,0,0,0,1;

    return SE3vel(inv);
}

SE3vel operator*(const SE3vel& lhs, const SE3vel& rhs)
{
    return SE3vel(Mat5(lhs.T()*rhs.T()));
}

Mat9 SE3vel::adj() const
{
    Mat9 res(Mat9::Zero());
    Mat3 R = this->R();
    Mat31 v = this->v();
    Mat31 t = this->t();

    res.block<3,3>(0,0) = R;
    res.block<3,3>(3,3) = R;
    res.block<3,3>(6,6) = R;

    res.block<3,3>(3,0) = hat3(v)*R;
    res.block<3,3>(6,0) = hat3(t)*R;

    return res;
}

std::ostream& operator<<(std::ostream &os, const SE3vel& obj)
    {
        os << obj.T();
        return os;
    }


Mat5 hat9(const Mat91 &xi)
{
    Mat5 result(Mat5::Zero());

    result <<    0, -xi(2),  xi(1), xi(3), xi(6),
             xi(2),      0, -xi(0), xi(4), xi(7),
            -xi(1),  xi(0),      0, xi(5), xi(8),
                 0,      0,      0,     0,     0,
                 0,      0,      0,     0,     0;

    return result;   
}


/**
 * Vee operator (v), the inverse of hat
 */
Mat91 vee9(const Mat4 &xi_hat)
{
    Mat91 result(Mat91::Zero());

    result << xi_hat(2,1), xi_hat(0,2), xi_hat(1,0),
              xi_hat(0,3), xi_hat(1,3), xi_hat(2,3),
              xi_hat(0,4), xi_hat(1,4), xi_hat(2,4);

    return result;
}


Mat3 SE3vel::left_jacobian(const Mat31& phi)
{
    Mat3 V = Mat3::Identity();
    Mat3 phi_hat = hat3(phi);
    double o = phi.norm();
    double o2 = phi.squaredNorm();
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
    V += c2*phi_hat + c3*phi_hat*phi_hat;

    return V;
}


Mat3 SE3vel::inv_left_jacobian(const Mat31 &phi)
{
    Mat3 Vinv = Mat3::Identity();
    double k1;
    double o = phi.norm();
    double o2 = phi.squaredNorm();
    Mat3 phi_hat = hat3(phi);
    // 5e-3 bound provided on numerical_test.cpp, smaller than this k1 becomes degradated
    if (o > 5e-3)
    {
        double c1 = std::sin(o); //sin(o)/o, we remove the o in both coeficients
        double c2 = (1 - std::cos(o))/o; // (1 - std::cos(o))/o/o
        k1 = 1/o2*(1 - 0.5*c1/c2);
    }
    //Taylor expansion for small o.
    else
    {
        // f(o) = 1/12 + 1/2*f''*o^2
        // f'' = 1/360
        k1 = 1.0/12 + o2/720;
    }
    Vinv += -0.5*phi_hat + k1* phi_hat*phi_hat;

    // v = V^-1 t
    return Vinv;
}

void SE3vel::Exp(const Mat91& xi)  
{
    Mat5 result(Mat5::Identity());

    Mat31 phi = xi.head(3);
    Mat31 v = xi.segment<3>(3);
    Mat31 t = xi.tail(3);

    SO3 tmp(phi);

    result.topLeftCorner<3,3>() << tmp.R();

    Mat3 jac = this->left_jacobian(phi);

    result.block<3,1>(0,3) << jac*v;
    result.block<3,1>(0,4) << jac*t;

    this->T_ = result;
}

Mat91 SE3vel::Ln() const
{
    Mat91 result;

    Mat3 R = this->R();
    Mat31 v = this->v();
    Mat31 t = this->t();

    SO3 tmp(R);
    Mat31 log_R_vee = tmp.ln_vee();
    Mat3 jac = SE3vel::inv_left_jacobian(log_R_vee);

    result.head(3) << log_R_vee;
    result.segment<3>(3) << jac*v;
    result.tail(3) << jac*t;

    return result;
}

void SE3vel::regenerate()
{
    Mat91 xi = this->Ln();
    this->Exp(xi);
}
