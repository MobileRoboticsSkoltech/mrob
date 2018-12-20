/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
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


#include "skmr/matrix_base.hpp"




/**
 *  \brief Special Orthogonal (group) in 3d
 *  Is the group representing rotations
 *  SO3 = {R \in GL(3) |  R'R = I , det R = 1}
 *  where GL(n) is the general linear group of matrices nxn (inverse exists)
 *  Associated to the group SO3, there is the Lie algebra so3
 *  representing the same transformation in the tangent space around the identity.
 *  Particularly, w  \in Re^3 represents the rotation
 */
namespace skmr{


class SO3 : public Mat3
{
public:
    /**
     * Constructor, requires the Lie algebra w \in so3 representing the
     * rotation around the identity, by default generates R = exp(0^) = I
     *
     */
    SO3(const Mat31 &w = Mat31::Zero());

    /**
     * This constructor allows to construct from Eigen expressions
     * Eigen suggestion: TopicCustomizingEigen.html
     */
    template<typename OtherDerived>
    SO3(const Eigen::MatrixBase<OtherDerived>& other);

    /**
     * This method allows to assign Eigen expressions to SO3
     * Eigen suggestion: TopicCustomizingEigen.html
     */
    template<typename OtherDerived>
    SO3& operator=(const Eigen::MatrixBase <OtherDerived>& other);
    /**
     * Updates the current transformation with the incremental dw \in so3
     * R'=exp(dw^)*R
     */
    void update(const Mat31 &dw);
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
    void print(void) const;
    void print_lie(void) const;

protected:
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

}// end namespace
#endif /* SO3_HPP_ */
