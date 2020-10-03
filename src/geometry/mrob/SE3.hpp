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
 * SE3.hpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#ifndef SE3_HPP_
#define SE3_HPP_


#include "mrob/matrix_base.hpp"
#include "mrob/SO3.hpp"




/**
 *  \brief Special Euclidean (group) in 3d
 *  Is the group representing rotations and translations, that is,
 *  rigid body transformations.
 *  SE3 = {T = [R  t]  |  R \in SO3 , t \in Re^3 }
 *             [0  1]
 *  Associated to the groups of RBT, there is the Lie algebra se3
 *  representing the same transformation in the tangent space around the identity.
 *  Particularly, xi =[w , v] \in Re^6, where w \in Re^3 represents the rotation
 *  and v the translation.
 *  We will preserve this order in this class.
 */
namespace mrob{


class SE3
{
public:
    /**
     * Constructor, requires the Transformation matrix 4x4
     */
    SE3(const Mat4 &T = Mat4::Identity() );
    /**
     * Constructor, requires the Lie algebra xi^ \in se3 representing the rigid body
     * transformation around the identity, by default generates T = exp(0^) = I
     */
    SE3(const Mat61 &xi);
    /**
     * Constructor, requires the Transformation in Se3
     */
    SE3(const SE3 &T);
    /**
     * Constructor, requires the Rotation in SO3 and translation
     */
    SE3(const SO3 &R, const Mat31 t);
    /**
     * Constructor, requires the Rotation as a 3x3 matrix and translation
     */
    SE3(const Mat3 &R, const Mat31 t);
    /**
     * This constructor allows to construct from Eigen expressions
     * Eigen suggestion: TopicCustomizingEigen.html
     */
    template<typename OtherDerived>
    SE3(const Eigen::MatrixBase<OtherDerived>& rhs);
    /**
     * This method allows you to assign Eigen expressions to SE3
     */
    SE3& operator=(const SE3& rhs);

    /**
     * This method allows you to Multiply SE3 expressions
     */
    SE3 operator*(const SE3& rhs) const;
    /**
     * Multiplication function, same as above, mainly for python
     */
    SE3 mul(const SE3& rhs) const;

    /**
     * This is our *default* way to update transformations, from the Left hand side of T
     * Updates the current transformation with the incremental dxi \in se3
     * T'=exp(dxi^) * T
     */
    void update_lhs(const Mat61 &dxi);
    /**
     * Updates the current transformation with the incremental dxi \in se3
     * T'= T * exp(dxi^)
     */
    void update_rhs(const Mat61 &dxi);
    /**
     *  Exponential mapping of a skew symetric matrix in se3.
     *  exp(xi^) = [exp(w^)  Vv], where exp(w^) is the so3_exp and
     *  V = I + c2*(w^) + c3*(w^)^2   , where o = norm(w), c2 = (1 - cos(o))/o^2, c3 = (o- sin(o) / o^3
     *  t= Vv
     *  Exponential mapping of a skew symetric matrix in so3. The Rodrigues formula provides
     *  an exact solution to the Taylor expansion of exp(A) = I + A + c2*A^2 + ...
     *  exp(A) = I + c1*w^ + c2*(w^)^2, where o = norm(w), c1 =sin(o)/o and c2 = (1 - cos(o))/o^2
     */
    void exp(const Mat4 &xi_hat);
    /**
     * Logarithm map, first we calculate ln(R) and then
     * V^-1 = I - 1/2 w^ + 1/o^2(1 - A / 2B) (w^)^2
     * v = V^-1 t
     */
    Mat4 ln(void) const;
    /**
     * Returns the vector xi \in R^6 which corresponds to the Lie algebra se3
     */
    Mat61 ln_vee() const;
    /**
     * Transforms a point p = (x,y,z)' such as res = T*p.
     * This function saves to transform to homogeneous coordinates.
     */
    Mat31 transform(const Mat31 & p) const;
    /**
     * Transforms an array of points P = {p_n} = (x,y,z)'_n such as res = T*p_n.
     * The array is of the form Nx3 (usual convention from arrays)
     * This function saves to transform to homogeneous coordinates.
     */
    MatX transform_array(const MatX &P) const;
    /**
     * Inverse: T^-1 = [R', -R't]
     *                 [0      1]
     */
    SE3 inv(void) const;
    /**
     * Adjoint: T Exp(x) = Exp ( Adj_T x) T
     *          Adj_T = [R ,   0]
     *                  [t^R,  R]
     */
    Mat6 adj() const;
    /**
     * T method returns a matrix 4x4 of the SE3 transformation. Ref<> is more convinient than
     * the matrix for the factor/nodes base class definitions and python bindings
     */
    //Mat4 T() const;
    const Eigen::Ref<const Mat4> T() const;
    /**
     * ref2T returns a non-const reference to the matrix T to modify its content directly
     */
    Mat4& ref2T();
    /**
     * R method returns a matrix 3x3 of the SO3 rotation corresponding to the subblock matrix
     */
    Mat3 R() const;
    /**
     * t method returns translation
     */
    Mat31 t() const;
    /**
     * Provide the distance as a norm on the tangent space
     * of the ln(T * T_rhs^{-1})
     */
    double distance(const SE3 &rhs=SE3()) const;
    /**
     * Provide the distance on the rotation in the tangent space
     * of the ln(R * R_rhs^{-1})
     */
    double distance_rotation(const SE3 &rhs=SE3()) const;
    /**
     * Provide the distance of the translation part
     * ||t - t'||
     */
    double distance_trans(const SE3 &rhs=SE3()) const;
    /**
     * Regenerate, does the following operation:
     * T = Exp ( Ln(T) )
     */
    void regenerate();


    void print(void) const;
    void print_lie(void) const;

protected:
    Mat4 T_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * Hat operator xi^ = [0  -w3   w2 v1
 *                     w3   0  -w1 v2
 *                   -w2  w1    0  v3
 *                    0    0    0   0]
 */
Mat4 hat6(const Mat61 &xi);
/**
 * Vee operator (v), the inverse of hat
 */
Mat61 vee6(const Mat4 &xi_hat);

bool isSE3(Mat4 T);

}// end namespace
#endif /* SE3_HPP_ */
