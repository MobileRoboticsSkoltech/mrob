/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
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


#include "skmr/matrix_base.hpp"




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
namespace skmr{


class SE3 : public Mat4
{
public:
    /**
     * Constructor, requires the Lie algebra xi \in se3 representing the rigid body
     * transformation around the identity, by default generates T = exp(0^) = I
     */
    SE3(const Mat61 &xi = Mat61::Zero());
    /**
     * This constructor allows you to construct from Eigen expressions
     * Eigen suggestion: TopicCustomizingEigen.html
     */
    template<typename OtherDerived>
    SE3(const Eigen::MatrixBase<OtherDerived>& other);

    /**
     * This method allows you to assign Eigen expressions to SE3
     * Eigen suggestion: TopicCustomizingEigen.html
     */
    template<typename OtherDerived>
    SE3& operator=(const Eigen::MatrixBase <OtherDerived>& other);

    /**
     * Updates the current transformation with the incremental dxi \in se3
     * T'=exp(dxi^)*T
     */
    void update(const Mat61 &dxi);
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
     * Logarithm map, first we calculate log(R) and then
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
     * Inverse: T^-1 = [R', -R't]
     *                 [0      1]
     */
    SE3 inv(void) const;
    /**
     * Adjoint: Adj_xi = [R , t^R]
     *                   [0    R]
     */
    Mat6 adj() const;
    void print(void) const;
    void print_lie(void) const;

protected:
};


/**
 * Hat operator xi^ = [0  -w3   w2 v1
 *                     w3   0  -w1 v2
 *                   -w2  w1    0  v3
 *                    0    0    0   0]
 */
Mat4 hat6(const Mat61 &xi);
/**
 * Vee operator, the inverse of hat
 */
Mat61 vee6(const Mat4 &xi_hat);

}// end namespace
#endif /* SE3_HPP_ */
