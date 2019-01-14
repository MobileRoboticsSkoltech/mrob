/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * gicp.hpp
 *
 *  Created on: Jan 31, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech 
 */

#ifndef GICP_HPP_
#define GICP_HPP_

#include "mrob/base_transf.hpp"

/**
 * Custom implementation of the GICP using SE3 optimization (improvement over Euler rotations)
 *
 * This method calculates the transformation
 *  T = [R t] between sets of points x in R^3 and y in R^3, such as:
 *      [0 1],
 *  where  yh = R*xh + t
 *
 *
 * T = min sum || y - Tx ||S
 *
 * The covariances provided are of the form S = R diag(e,1,1) R', so they MUST have been already processed.
 */


namespace mrob{

class GICP:  public BaseTransf {
  public:
    GICP(const MatX &X, const MatX &Y, const MatX &CovX, const MatX &CovY);
    virtual ~GICP();
    virtual int solve();

  protected:
    const MatX &CovX_, &CovY_;
};

}//end namespace


#endif /* GICP_HPP_ */
