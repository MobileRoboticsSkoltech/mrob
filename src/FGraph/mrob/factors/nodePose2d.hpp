/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 *  Created on: Jan 14, 2019
 *      Author: Konstantin Pakulev
 *              konstantin.pakulev@skoltech.ru
 *              Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef MROB_NODEPOSE2D_H
#define MROB_NODEPOSE2D_H

#include "mrob/matrix_base.hpp"
#include "mrob/node.hpp"

namespace mrob{


    /**
     * This class NodePose2d for now is parametrized on XYT coordinates
     * TODO compare with SE(2)
     */
    class NodePose2d : public Node {
    public:
        /**
         * For initialization, requires an initial estimation of the state.
         */
        explicit NodePose2d(const Mat31 &initial_x);
        virtual ~NodePose2d() = default;

        void update(const Eigen::Ref<const MatX1> &dx);
        virtual const Eigen::Ref<const MatX1> getState() const {return x_;};
        void print() const;
    protected:
        Mat31 x_;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

#endif //MROB_NODEPOSE2D_H
