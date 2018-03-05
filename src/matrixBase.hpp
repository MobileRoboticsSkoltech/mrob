/*
 * matrix_base.hpp
 *
 *  Created on: Jun 21, 2017
 *      Author: gonzalo
 */

#ifndef MATRIX_BASE_HPP_
#define MATRIX_BASE_HPP_

#include <Eigen/Dense>


// data types conventions
typedef double matData_t;
typedef unsigned int uint_t;


// Definition of squared matrices
typedef Eigen::Matrix<matData_t, 2,2> Mat2;
typedef Eigen::Matrix<matData_t, 3,3> Mat3;
typedef Eigen::Matrix<matData_t, 4,4> Mat4;
typedef Eigen::Matrix<matData_t, 5,5> Mat5;
typedef Eigen::Matrix<matData_t, 6,6> Mat6;
typedef Eigen::Matrix<matData_t, Eigen::Dynamic,Eigen::Dynamic> MatX;


// Definition of column matrices (vectors)
typedef Eigen::Matrix<matData_t, 2,1> Mat21;
typedef Eigen::Matrix<matData_t, 3,1> Mat31;
typedef Eigen::Matrix<matData_t, 4,1> Mat41;
typedef Eigen::Matrix<matData_t, 5,1> Mat51;
typedef Eigen::Matrix<matData_t, 6,1> Mat61;






#endif /* MATRIX_BASE_HPP_ */
