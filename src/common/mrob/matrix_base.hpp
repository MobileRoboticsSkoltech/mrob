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
 * time_profiling.jpp
 *
 *  Created on: Aug 14, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab.
 */

#ifndef MATRIX_BASE_HPP_
#define MATRIX_BASE_HPP_

//#include <Eigen/Dense> // overkill including everything, Specific dependencies will be set at each module
#include <Eigen/Core>
//#include <Eigen/LU> // for inverse and determinant
#include <Eigen/Sparse>

#define EIGEN_DEFAULT_TO_ROW_MAJOR

// data types conventions
typedef double matData_t;
typedef unsigned int uint_t;


// Definition of squared matrices, by default column major
typedef Eigen::Matrix<matData_t, 1,1, Eigen::RowMajor> Mat1;
typedef Eigen::Matrix<matData_t, 2,2, Eigen::RowMajor> Mat2;
typedef Eigen::Matrix<matData_t, 3,3, Eigen::RowMajor> Mat3;
typedef Eigen::Matrix<matData_t, 4,4, Eigen::RowMajor> Mat4;
typedef Eigen::Matrix<matData_t, 5,5, Eigen::RowMajor> Mat5;
typedef Eigen::Matrix<matData_t, 6,6, Eigen::RowMajor> Mat6;
typedef Eigen::Matrix<matData_t, 9,9, Eigen::RowMajor> Mat9;
typedef Eigen::Matrix<matData_t, Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajor> MatX;

//Sparse Matrices
typedef Eigen::SparseMatrix<matData_t, Eigen::ColMajor> SMatCol;
typedef Eigen::SparseMatrix<matData_t, Eigen::RowMajor> SMatRow;
typedef Eigen::Triplet<matData_t> Triplet;

// Definition of column matrices (vectors)
typedef Eigen::Matrix<matData_t, 2,1> Mat21;
typedef Eigen::Matrix<matData_t, 3,1> Mat31;
typedef Eigen::Matrix<matData_t, 4,1> Mat41;
typedef Eigen::Matrix<matData_t, 5,1> Mat51;
typedef Eigen::Matrix<matData_t, 6,1> Mat61;
typedef Eigen::Matrix<matData_t, 9,1> Mat91;
typedef Eigen::Matrix<matData_t, Eigen::Dynamic,1> MatX1;

// Definition of row matrices (vectors)
typedef Eigen::Matrix<matData_t, 1,2> Mat12;
typedef Eigen::Matrix<matData_t, 1,3> Mat13;
typedef Eigen::Matrix<matData_t, 1,4> Mat14;
typedef Eigen::Matrix<matData_t, 1,5> Mat15;
typedef Eigen::Matrix<matData_t, 1,6> Mat16;
typedef Eigen::Matrix<matData_t, 1,9> Mat19;
typedef Eigen::Matrix<matData_t, 1,Eigen::Dynamic> Mat1X;


// Definition of templated-based fixed matrices using c'11 aliases
template<int D>
using Vect = Eigen::Matrix<matData_t, D,1, Eigen::RowMajor>;
template<int Rw,int Col>
using Mat = Eigen::Matrix<matData_t, Rw, Col, Eigen::RowMajor>;



#endif /* MATRIX_BASE_HPP_ */
