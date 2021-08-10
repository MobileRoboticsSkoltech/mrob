#include "mrob/SE3cov.hpp"
#include <iostream>

using namespace mrob;

SE3Cov::SE3Cov(void): SE3(),covariance_(Mat6::Identity()){}

SE3Cov::SE3Cov(const SE3& T, const Mat6 &cov): SE3(T), covariance_(cov){}

SE3Cov::SE3Cov(const SE3Cov& pose):SE3(Mat4(pose.T())), covariance_(pose.cov()){}

Mat3 brackets(const Mat3& A)
{
    return -A.trace()*Mat3::Identity() + A;
}

Mat3 brackets(const Mat3& A, const Mat3& B)
{
    return brackets(A)*brackets(B) + brackets(B*A);
}

Mat6 SE3Cov::cov(void) const
{
    return this->covariance_;
}


SE3Cov SE3Cov::compound_2nd_order(const SE3 &pose_increment, const Mat6 &increment_covariance) const
{
    // Gonzalo: I think this method should return a copy and not modify itself, same as in SE3
    Mat6 adj = this->adj();
    return SE3Cov(SE3::mul(pose_increment), covariance_ + adj*increment_covariance*adj.transpose());
}

SE3Cov SE3Cov::compound_2nd_order(const SE3Cov& pose) const
{
    return compound_2nd_order((const SE3)pose, pose.cov());
}


SE3Cov SE3Cov::compound_4th_order(const SE3 &pose_increment, const Mat6 &increment_covariance) const
{
    Mat6 sigma_1 = covariance_;

    Mat6 adj = this->adj();
    Mat6 sigma_2 = adj*increment_covariance*adj.transpose();
 
    //Calculating the covariance update, correction to the mrob convention xi = [theta, rho]
    Mat6 A_1(Mat6::Zero());
    Mat3 sigma_1_tt = sigma_1.topLeftCorner<3,3>();//theta t (instead of phi)
    Mat3 sigma_1_rr = sigma_1.bottomRightCorner<3,3>();
    Mat3 sigma_1_rt = sigma_1.bottomLeftCorner<3,3>();
    Mat3 sigma_1_tr = sigma_1.topRightCorner<3,3>();
    A_1.topLeftCorner<3,3>() = brackets(sigma_1_tt);
    A_1.bottomLeftCorner<3,3>() = brackets(sigma_1_rt + sigma_1_tr);
    A_1.bottomRightCorner<3,3>() = A_1.topLeftCorner<3,3>();

    std::cout << A_1 << std::endl;

    Mat6 A_2(Mat6::Zero());
    Mat3 sigma_2_tt = sigma_2.topLeftCorner<3,3>();
    Mat3 sigma_2_rr = sigma_2.bottomRightCorner<3,3>();
    Mat3 sigma_2_rt = sigma_2.bottomLeftCorner<3,3>();
    Mat3 sigma_2_tr = sigma_2.topRightCorner<3,3>();
    A_2.topLeftCorner<3,3>() = brackets(sigma_2_tt);
    A_2.bottomLeftCorner<3,3>() = brackets(sigma_2_rt + sigma_2_rt.transpose());
    A_2.bottomRightCorner<3,3>() = A_2.topLeftCorner<3,3>();

    std::cout << A_2 << std::endl;

    Mat6 B(Mat6::Zero());

    Mat3 B_rho_rho = brackets(sigma_1_tt,sigma_2_rr) +
                     brackets(sigma_1_tr,sigma_2_rt) +
                     brackets(sigma_1_rt, sigma_2_tr) +
                     brackets(sigma_1_rr, sigma_2_tt);

    Mat3 B_rho_phi = brackets(sigma_1_tt, sigma_2_tr) +
                     brackets(sigma_1_rt, sigma_2_tt);// There is a mistake in Barfoots p.265,

    Mat3 B_phi_phi = brackets(sigma_1_tt, sigma_2_tt);

    B.topLeftCorner<3,3>() = B_phi_phi;
    B.topRightCorner<3,3>() = B_rho_phi.transpose();
    B.bottomLeftCorner<3,3>() = B_rho_phi;
    B.bottomRightCorner<3,3>() = B_rho_rho;

    std::cout << B << std::endl;

    Mat6 tmp_cov = sigma_1 + sigma_2 +
                        1./12.*(A_1*sigma_2 + sigma_2*A_1.transpose() + A_2*sigma_1 + sigma_1*A_2.transpose())+
                        1./4.*B;


    // calculating the resulting pose
    return SE3Cov(SE3::mul(pose_increment), tmp_cov);
}

SE3Cov SE3Cov::compound_4th_order(const SE3Cov& pose) const
{
    return compound_4th_order((const SE3)pose, pose.cov());
}

void SE3Cov::print()
{
    std::cout << "Pose:" << std::endl;
    std::cout << this->T_ << std::endl;
    std::cout << "Covariance:" << std::endl;
    std::cout << this->cov() << std::endl;
}

SE3Cov SE3Cov::operator*(const SE3Cov& rhs) const
{
    return SE3Cov::compound_2nd_order(rhs);
}

Mat6 mrob::curly_wedge(const Mat61& xi)
{
    Mat6 result(Mat6::Zero());
    result.topLeftCorner<3,3>() = mrob::hat3(xi.head(3));
    result.bottomRightCorner<3,3>() = mrob::hat3(xi.head(3));
    result.bottomLeftCorner<3,3>() = mrob::hat3(xi.tail(3));
    return result;
}
