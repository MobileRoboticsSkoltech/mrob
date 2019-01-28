//
// Created by Konstantin on 14/01/2019.
//

#ifndef MROB_FACTOR1POSE2D_H
#define MROB_FACTOR1POSE2D_H

#include "mrob/matrix_base.hpp"
#include "mrob/factor.hpp"

using namespace mrob;

namespace mrob{
    class Factor1Pose2d : public Factor{
    public:
        Factor1Pose2d(const Mat31 &observation, std::shared_ptr<Node> &n1,
                const Mat3 &obsInf);
        ~Factor1Pose2d() = default;
        /**
         * Evaluates residuals and Jacobians
         */
        void evaluate();

        /**
         * Returns the chi2 error and fills the residual vector
         */
        matData_t evaluateError();

        void print() const;

        const Eigen::Ref<const MatX1> getObs() const {return obs_;};
        const Eigen::Ref<const MatX1> getResidual() const {return r_;};
        const Eigen::Ref<const MatX> getInvCovariance() const {return W_;};
        const Eigen::Ref<const MatX> getWT2() const{return WT2_;};
        const Eigen::Ref<const MatX> getJacobian() const {return J_;};

    protected:
        Mat31 obs_, r_; //and residuals
        Mat3 W_;//inverse of observation covariance (information matrix)
        Mat3 WT2_;//transpose and squared root of W. We could delete this variable...
        Mat3 J_;//Jacobian
    };
}

#endif //MROB_FACTOR1POSE2D_H
