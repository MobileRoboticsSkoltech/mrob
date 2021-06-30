#ifndef SE3VEL_HPP_
#define SE3VEL_HPP_

#include <iostream>

#include "mrob/matrix_base.hpp"
#include "mrob/SO3.hpp"

namespace mrob{

class SE3vel{
    public:
        SE3vel(const Mat5 &T = Mat5::Identity());

        SE3vel(const SE3vel &T);

        SE3vel(const Mat91 &xi);

        SE3vel inv(void) const;

        Mat31 t() const;
        Mat31 v() const;
        Mat3 R() const;
        Mat5 T() const;

        Mat9 adj() const;

        static Mat3 left_jacobian(const Mat31 &phi);
        static Mat3 inv_left_jacobian(const Mat31 &phi);

        void Exp(const Mat91 &xi);
        Mat91 Ln(void) const;

        void regenerate();        
    protected:
        Mat5 T_;
};

Mat5 hat9(const Mat91 &xi);

Mat91 vee9(const Mat5 &xi_hat);

}// end namespace

mrob::SE3vel operator*(const mrob::SE3vel& lhs, const mrob::SE3vel& rhs);
std::ostream& operator<<(std::ostream &os, const mrob::SE3vel &obj);

#endif /* SE3VEL_HPP_ */
