/* $COPYRIGHT_SKOLTECH
 * $LICENSE_LGPL
 *
 * matrix_base.hpp
 *
 *  Created on: Feb 21, 2018
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 *
 */

#ifndef MATRIX_BASE_HPP_
#define MATRIX_BASE_HPP_

//#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Sparse>

// data types conventions
typedef double matData_t;
typedef unsigned int uint_t;
typedef unsigned int id_t;


// Definition of squared matrices
typedef Eigen::Matrix<matData_t, 2,2> Mat2;
typedef Eigen::Matrix<matData_t, 3,3> Mat3;
typedef Eigen::Matrix<matData_t, 4,4> Mat4;
typedef Eigen::Matrix<matData_t, 5,5> Mat5;
typedef Eigen::Matrix<matData_t, 6,6> Mat6;
typedef Eigen::Matrix<matData_t, Eigen::Dynamic,Eigen::Dynamic> MatX;

//Sparse Matrices
typedef Eigen::SparseMatrix<matData_t, Eigen::ColMajor> SMat;
typedef Eigen::SparseMatrix<matData_t, Eigen::RowMajor> SMatRow;
typedef Eigen::Triplet<matData_t> Triplet;

// Definition of column matrices (vectors)
typedef Eigen::Matrix<matData_t, 2,1> Mat21;
typedef Eigen::Matrix<matData_t, 3,1> Mat31;
typedef Eigen::Matrix<matData_t, 4,1> Mat41;
typedef Eigen::Matrix<matData_t, 5,1> Mat51;
typedef Eigen::Matrix<matData_t, 6,1> Mat61;
typedef Eigen::Matrix<matData_t, Eigen::Dynamic,1> MatX1;


// Definition of templated-based fixed matrices using c'11 aliases
template<int D>
using MatD1 = Eigen::Matrix<matData_t, D,1>;
template<int Rw,int Col>
using MatD = Eigen::Matrix<matData_t, Rw, Col>;



#endif /* MATRIX_BASE_HPP_ */
