//
// Created by Konstantin on 14/01/2019.
//

#ifndef MROB_FACTOR2POSES2D_H
#define MROB_FACTOR2POSES2D_H

#include "mrob/matrix_base.hpp"
#include "mrob/factor.hpp"


namespace mrob{

    class Factor2Poses2d : public Factor {
    public:
        Factor2Poses2d(const Mat31 &observation, std::shared_ptr<Node> &n1,
                       std::shared_ptr<Node> &n2, const Mat3 &obsInf);
        ~Factor2Poses2d() override = default;

        void print() const override;

        const Eigen::Ref<const MatX1> getObs() const override {return obs_;};
        const Eigen::Ref<const MatX1> getResidual() const override {return r_;};
        const Eigen::Ref<const MatX> getInvCovariance() const override {return W_;};
        const Eigen::Ref<const MatX> getWT2() const override {return WT2_;};
        const Eigen::Ref<const MatX> getJacobian() const override {return J_;};

    protected:
        double wrap_angle(double angle);
        // The Jacobian's correspondent nodes are ordered on the vector<Node>
        // being [0]->J1 and [1]->J2
        // declared here but initialized on child classes
        Mat31 obs_, r_; //and residuals
        Mat3 W_;//inverse of observation covariance (information matrix)
        Mat3 WT2_;//transpose and squared root of W.
        MatD<3,6> J_;//Joint Jacobian

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen
    };
}

#endif //MROB_FACTOR2POSES2D_H
