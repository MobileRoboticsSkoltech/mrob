#ifndef SE3VELCOV_HPP_
#define SE3VELCOV_HPP_

#include "mrob/SE3vel.hpp"

namespace mrob
{
    class SE3velCov : public mrob::SE3vel
    {
    public:
        SE3velCov(void);
        SE3velCov(const SE3vel &pose, const Mat9 &covariance);
        Mat9 covariance;
        void compound_2nd_order(const SE3vel &pose_increment, const Mat9 &increment_covariance, const double dt);
        void compound_4th_order(const SE3vel &pose_increment, const Mat9 &increment_covariance, const double dt);

        Mat9 getQ(const Mat3& cov_a, const Mat3& cov_w, const double dt) const;

        Mat9 get_S_4th(const Mat9 &Q) const;

        void print();

        // transforms covariance matrix to notation from Barfoot's papers
        // transform(transform(cov)) = cov - self-inverse
        Mat9 transform(const Mat9 &covariance) const;
    };

    // Mat3 brackets(const Mat3 &A);
    // Mat3 brackets(const Mat3 &A, const Mat3 &B);

} // end namespace

#endif //SE3VELCOV_HPP_
