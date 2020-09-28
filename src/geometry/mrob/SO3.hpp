/* Copyright (c) 2018, Skolkovo Institute of Science and Technology (Skoltech)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * SO3.hpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#ifndef SO3_HPP_
#define SO3_HPP_


#include "mrob/matrix_base.hpp"




/**
 *  \brief Special Orthogonal (group) in 3d
 *  Is the group representing rotations
 *  SO3 = {R \in GL(3) |  R'R = I , det R = 1}
 *  where GL(n) is the general linear group of matrices nxn (inverse exists)
 *  Associated to the group SO3, there is the Lie algebra so3
 *  representing the same transformation in the tangent space around the identity.
 *  Particularly, w  \in Re^3 represents the rotation
 *
 *  In addition, we have added some utilities regarding alternative methods
 *  to build SO(3) and connect them to the manifold representation.
 */
namespace mrob{


class SO3
{
public:
    /**
     * Constructor from a Matrix transformation
     *
     */
    SO3(const Mat3 &R = Mat3::Identity());
    /**
     * Constructor, requires the Lie algebra w \in so3 representing the
     * rotation around the identity, by default generates R = exp(0^) = I
     *
     */
    SO3(const Mat31 &w);
    /**
     * Constructor from an SO3 transformation
     *
     */
    SO3(const SO3 &R);
    /**
     * This constructor allows to construct from Eigen expressions
     * Eigen suggestion: TopicCustomizingEigen.html
     */
    template<typename OtherDerived>
    SO3(const Eigen::MatrixBase<OtherDerived>& rhs);
    /**
     * This method allows you to assign Eigen expressions to SO3
     */
    SO3& operator=(const SO3& rhs);

    /**
     * This method allows you to Multiply SO3 expressions
     */
    SO3 operator*(const SO3& rhs) const;

    /**
     * multiplication function
     */
    SO3 mul(const SO3& rhs) const;

    /**
     * This is our *default* way to update transformations
     * Updates the current transformation with the incremental dw \in so3
     * R'=exp(dw^)*R
     */
    void update_lhs(const Mat31 &dw);
    /**
     * Updates the current transformation with the incremental dw \in so3
     * R'=R*exp(dw^)
     */
    void update_rhs(const Mat31 &dw);
    /**
     *  Exponential mapping of a skew symetric matrix in so3. The Rodrigues formula provides
     *  an exact solution to the Taylor expansion of exp(A) = I + A + c2*A^2 + ...
     *  exp(A) = I + c1*w^ + c2*(w^)^2, where o = norm(w), c1 =sin(o)/o and c2 = (1 - cos(o))/o^2
     */
    void exp(const Mat3 &w_hat);
    /**
     * Logarithmic mapping from SO3 to a skew-symetric matrix in so3
     * o = |acos(0.5*(tr(R)-1))|
     * w^ = o / (2sin(o))*(R-R^T)
     *
     * Special cases for 0 and +-pi
     */
    Mat3 ln(double *o = nullptr) const;
    /**
     * Logarithm of R and then the vee operator to get the coordinates w of the Lie Algebra
     */
    Mat31 ln_vee() const;
    /**
     * Inverse: R^-1 = R'
     */
    SO3 inv(void) const;
    /**
     * Adjoint: Adj_w = R
     */
    Mat3 adj() const;
    /**
     * R method returns the matrix 3x3 of the SO3 rotation
     */
    Mat3 R() const;
    /**
     * ref to R method returns the reference to modify R
     */
    Mat3& ref2R();

    double distance(const SO3 &rhs) const;

    void print(void) const;
    void print_lie(void) const;


protected:
    Mat3 R_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * Functions specified outside the class, since they are not strictly necessary as a class method.
 *
 * The hat operator (^) constructs a skew-symetric matrix from a R^3 such as [0 -w3 w2]
 *                                                                           [w3  0 -w1]
 *                                                                           [-w2 w1 0]
 */
Mat3 hat3(const Mat31 &w);
/**
 * The vee operator (v) gets the parameters w from a skew-symetric matrix of the form [0 -w3 w2]
 *                                                                                [w3  0 -w1]
 *                                                                                [-w2 w1 0]
 */
Mat31 vee3(const Mat3 &w_hat);

bool isSO3(Mat3 R);


/**
 * Function converting from quaternion q = [qx, qy, qz, qw](Eigen convention)
 * to a rotation matrix 3x3
 * XXX: ref eigen did not return a valid matrix (probably lifetime was managed from cpp and this object was local to this scope)
 */
Mat3 quat_to_so3(const Eigen::Ref<const Mat41> v);


/**
 * Function converting from a rotation matrix 3x3 to quaternion
 * q = [qx, qy, qz, qw](Eigen convention)
 */
Mat41 so3_to_quat(const Eigen::Ref<const Mat3> R);

/**
 * Function converting from roll pitch yaw v = [r, p, y](Eigen convention)
 * to a rotation matrix 3x3
 */
Mat3 rpy_to_so3(const Eigen::Ref<const Mat31> v);

}// end namespace
#endif /* SO3_HPP_ */
