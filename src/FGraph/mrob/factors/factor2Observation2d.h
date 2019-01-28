//
// Created by Konstantin on 17/01/2019.
//

#ifndef MROB_FACTOR2OBSERVATION2D_H
#define MROB_FACTOR2OBSERVATION2D_H

#include "mrob/factors/factor2Poses2d.h"

using namespace mrob;

namespace mrob{

    class Factor2Observation2d : public Factor2Poses2d {
    public:
        Factor2Observation2d(const Mat31 &observation, std::shared_ptr<Node> &n1,
                          std::shared_ptr<Node> &n2, const Mat3 &obsInf);
        ~Factor2Observation2d() override = default;

        /**
         * Evaluates residuals and Jacobians
        */
        void evaluate() override;
        /**
         * Jacobians are not evaluated, just the residuals
         */
        matData_t evaluateError() override;
    };
}

#endif //MROB_FACTOR2OBSERVATION2D_H
