#ifndef SE3COV_HPP_
#define SE3COV_HPP_

#include "mrob/matrix_base.hpp"
#include "mrob/SE3.hpp"

namespace mrob
{
    class SE3Cov : public mrob::SE3
    {
    public:
        SE3Cov(void);
        SE3Cov(const SE3 &pose, const Mat6 &covariance);
        Mat6 covariance;

        void compound_2nd_order(const SE3Cov& pose);
        void compound_2nd_order(const SE3 &pose_increment, const Mat6 &increment_covariance); // does right hand side update


        void compound_4th_order(const SE3 &pose_increment, const Mat6 &increment_covariance);

        void print();

        SE3Cov mul(const SE3Cov& rhs) const; // keep this interface

        // transforms covariance matrix to notation from Barfoot's papers
        // transform(transform(cov)) = cov - self-inverse
        Mat6 transform(const Mat6 &covariance) const;
    };

} // end namespace

#endif // SE3COV_HPP_