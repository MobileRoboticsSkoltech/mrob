/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * SE3.hpp
 *
 *  Created on: Jan 21, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#ifndef MROBPY_SE3PY_HPP_
#define MROBPY_SE3PY_HPP_


#include "mrob/SO3.hpp"
#include "mrob/SE3.hpp"
using namespace mrob;

/**
 * Class to overcome the Eigen templated constructor, which does work.
 */
class PySO3 {
  public:
    PySO3(const Mat31 &w) : R_(w) { };
    PySO3(const Mat3 &R) : R_(R) { }; //this constructor in Python will result in a copy
    Mat3 R() {return (Mat3)R_;};
    void update(const Mat31 &dw) {R_.update(dw);};
    Mat31 ln() {return R_.ln_vee();};
    PySO3 inv(){return PySO3(R_.inv());}
    Mat3 adj(){return R_.adj();}
    //TODO add multiplication operator?

  protected:
    SO3 R_;

};


/**
 * Class to overcome the Eigen templated constructor, which does work.
 */
class PySE3 {
  public:
    PySE3(const Mat61 xi) : T_(xi) { };
    PySE3(const Mat4 &T) : T_(T) { }; //this constructor in Python will result in a copy
    Mat4 T() {return (Mat4)T_;};
    Mat3 R() {return (Mat3)T_.R();}
    Mat31 t() {return T_.t();}
    void update(const Mat61 &dxi) {T_.update(dxi);};
    Mat61 ln() {return T_.ln_vee();};
    Mat31 transform(const Mat31 &p) {return T_.transform(p); }
    MatX transformArray(const MatX &p) {return T_.transformArray(p); }
    PySE3 inv(){return PySE3(T_.inv());}
    Mat6 adj(){return T_.adj();}
    //TODO add multiplication operator?

  protected:
    SE3 T_;

};



#endif /* MROBPY_SE3PY_HPP_ */
