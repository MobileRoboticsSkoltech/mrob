#include "mrob/SE3cov.hpp"
#include <iostream>

using namespace mrob;

SE3Cov::SE3Cov(void)
{
    this->T_ = Mat4::Identity();
    this->covariance = Mat6::Identity();
}

SE3Cov::SE3Cov(const SE3& pose, const Mat6 &covariance)
{
    this->T_ = pose.T();
    this->covariance = covariance;
}

Mat3 brackets(const Mat3& A)
{
    return -A.trace()*Mat3::Identity() + A;
}

Mat3 brackets(const Mat3& A, const Mat3& B)
{
    return brackets(A)*brackets(B) + brackets(B*A);
}

Mat6 SE3Cov::transform(const Mat6& covariance) const
{
    Mat6 result(Mat6::Zero());
    result.topLeftCorner<3,3>() = covariance.bottomRightCorner<3,3>();
    result.bottomRightCorner<3,3>() = covariance.topLeftCorner<3,3>();
    result.topRightCorner<3,3>() = covariance.bottomLeftCorner<3,3>();
    result.bottomLeftCorner<3,3>() = covariance.topRightCorner<3,3>();
    return result;
}

void SE3Cov::compound_2nd_order(const SE3 &pose_increment, const Mat6 &increment_covariance)
{
    Mat6 adj = this->adj();

    this->T_ = this->T() * pose_increment.T();

    this->covariance = transform(transform(this->covariance) + adj*transform(increment_covariance)*adj.transpose());
}

void SE3Cov::compound_2nd_order(const SE3Cov& pose)
{
    compound_2nd_order((const SE3)pose, pose.covariance);
}


void SE3Cov::compound_4th_order(const SE3 &pose_increment, const Mat6 &increment_covariance)
{
    // moving to Barfoot notaion
    Mat6 sigma_1 = this->transform(this->covariance);

    Mat6 adj = this->adj();
    Mat6 sigma_2 = adj*transform(increment_covariance)*adj.transpose();
 
    //Calculating the covariance update according to the Barfoot article
    Mat6 A_1(Mat6::Zero());
    A_1.topLeftCorner<3,3>() = brackets(sigma_1.bottomRightCorner<3,3>());
    A_1.topRightCorner<3,3>() = brackets(sigma_1.topRightCorner<3,3>() + sigma_1.bottomLeftCorner<3,3>());
    A_1.bottomRightCorner<3,3>() = A_1.topLeftCorner<3,3>();

    Mat6 A_2(Mat6::Zero());
    A_2.topLeftCorner<3,3>() = brackets(sigma_2.bottomRightCorner<3,3>());
    A_2.topRightCorner<3,3>() = brackets(sigma_2.topRightCorner<3,3>() + sigma_2.bottomLeftCorner<3,3>());
    A_2.bottomRightCorner<3,3>() = A_2.topLeftCorner<3,3>();

    Mat6 B(Mat6::Zero());

    Mat3 B_rho_rho = brackets(sigma_1.bottomRightCorner<3,3>(),sigma_2.topLeftCorner<3,3>()) + 
                brackets(sigma_1.bottomLeftCorner<3,3>(),sigma_2.topRightCorner<3,3>()) +
                brackets(sigma_1.topRightCorner<3,3>(), sigma_2.bottomLeftCorner<3,3>()) +
                brackets(sigma_1.topLeftCorner<3,3>(), sigma_2.bottomRightCorner<3,3>());

    Mat3 B_rho_phi = brackets(sigma_1.bottomRightCorner<3,3>(), sigma_2.bottomLeftCorner<3,3>()) +
                brackets(sigma_1.bottomLeftCorner<3,3>(), sigma_2.bottomRightCorner<3,3>());

    Mat3 B_phi_phi = brackets(sigma_1.bottomRightCorner<3,3>(), sigma_2.bottomRightCorner<3,3>());

    B.topLeftCorner<3,3>() = B_rho_rho;
    B.topRightCorner<3,3>() = B_rho_phi;
    B.bottomLeftCorner<3,3>() = B_rho_phi.transpose();
    B.bottomRightCorner<3,3>() = B_phi_phi;

    this->covariance = sigma_1 + sigma_2 + 
    1./12.*(A_1*sigma_2 + sigma_2*A_1.transpose() + A_2*sigma_1 + sigma_1*A_2.transpose())+
    1./4.*B;

    // moving from Barfoot covariance notation into the mrob notation
    this->covariance = transform(this->covariance);

    // calculating the resulting pose
    this->T_ = this->T()*pose_increment.T();
}

void SE3Cov::print()
{
    std::cout << "Pose:" << std::endl;
    std::cout << this->T_ << std::endl;
    std::cout << "Covariance:" << std::endl;
    std::cout << this->covariance << std::endl;
}