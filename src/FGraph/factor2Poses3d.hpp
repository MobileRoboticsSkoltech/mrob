/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * factor2Poses3d.hpp
 *
 *  Created on: Feb 28, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef FACTOR2POSES3D_HPP_
#define FACTOR2POSES3D_HPP_


#include "factor.hpp"
#include <Eigen/Dense>
#include "matrixBase.hpp"
#include "SE3.hpp" //requires including and linking SE3 library
#include <unordered_map>

namespace fg{

class Factor2Poses3d : public Factor
{
  public:
    /**
     * For initialization, requires an initial estimation of the state.
     */
    Factor2Poses3d(unsign_t id, const Mat61 &observation, const Mat6 &obsCov);
    virtual ~Factor2Poses3d();
    virtual int getDim() const {return dim_;};
    /**
     * Evaluates residuals and Jacobians
     */
    void evaluate();
    /**
     * Jacobians are not evaluated, just the residuals
     */
    void evaluateLazy();
    Mat61 getObs() const {return obs_;};
    Mat6 getJacobian(unsign_t nodeId) const;
    Mat6 getCovariance() const {return obsCov_;};

  protected:
    int dim_;//fixed to 6, a RBT
    Mat61 obs_;
    lie::SE3 Tobs_;
    Mat6 obsCov_;
    // This clearly does not scales well, but we should not expect facors greater than 3...
    //std::unordered_map<unsign_t , std::shared_ptr<Mat6> > Jacobians_;
    unsign_t id1_, id2_;
    Mat6 J1_, J2_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // as proposed by Eigen

};


}




#endif /* FACTOR2POSES3D_HPP_ */
