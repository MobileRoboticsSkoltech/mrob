/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * arun.hpp
 *
 *  Created on: Jan 31, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef ARUN_HPP_
#define ARUN_HPP_

#include "base_T.hpp"

/**
 *  This class provides a set of methods to calculate the transformation
 *  T = [R t] between sets of points x in R^3 and y in R^3, such as:
 *      [0 1],
 *  where  yh = T*xh and
 *  being xh and yh the homogeneous description of those points [x y z 1].
 *
 *  In particular, This solution is based on the paper by Arun et al. "Least-squares fitting of two 3-D point sets", 1987
 *  Given N points x = x_1,x_2,x_3 and their correspondences y_i, calculate the
 *  transformation T = [R t], such as: min sum (y_i - (R * x_i + t))^2
 *
 *  An improvement over it was proposed by Sinji Umeyama 1991, which uniqueness determine R
 *
 *  3 points in x and y are the minimum requirements, and provide the right solution IF
 *  there is no noise.
 *
 *  Returns 0 if failed and 1 if a correct solution was found
 *
 *
 */
namespace align_mth{

class Carun:  public align_mth::base_T {
  public:
    Carun(const std::shared_ptr<MatX> &X, const std::shared_ptr<MatX> &Y);
    virtual ~Carun();
    virtual int solve();

  protected:
};

}//end namespace
#endif /* ARUN_HPP_ */
