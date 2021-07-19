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



namespace mrob{

// data types conventions
using matData_t = double;
using uint_t = unsigned int;
using factor_id_t = std::size_t;

// Definition of squared matrices, by default column major
using Mat1 = Eigen::Matrix<matData_t, 1,1, Eigen::RowMajor>;
using Mat2 = Eigen::Matrix<matData_t, 2,2, Eigen::RowMajor>;
using Mat3 = Eigen::Matrix<matData_t, 3,3, Eigen::RowMajor>;
using Mat4 = Eigen::Matrix<matData_t, 4,4, Eigen::RowMajor>;
using Mat5 = Eigen::Matrix<matData_t, 5,5, Eigen::RowMajor>;
using Mat6 = Eigen::Matrix<matData_t, 6,6, Eigen::RowMajor>;
using MatX = Eigen::Matrix<matData_t, Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajor>;

//Sparse Matrices
using SMatCol = Eigen::SparseMatrix<matData_t, Eigen::ColMajor>;
using SMatRow = Eigen::SparseMatrix<matData_t, Eigen::RowMajor>;
using Triplet = Eigen::Triplet<matData_t>;

// Definition of column matrices (vectors)
using Mat21 = Eigen::Matrix<matData_t, 2,1>;
using Mat31 = Eigen::Matrix<matData_t, 3,1>;
using Mat41 = Eigen::Matrix<matData_t, 4,1>;
using Mat51 = Eigen::Matrix<matData_t, 5,1>;
using Mat61 = Eigen::Matrix<matData_t, 6,1>;
using MatX1 = Eigen::Matrix<matData_t, Eigen::Dynamic,1>;

// Definition of row matrices (vectors)
using Mat12 = Eigen::Matrix<matData_t, 1,2>;
using Mat13 = Eigen::Matrix<matData_t, 1,3>;
using Mat14 = Eigen::Matrix<matData_t, 1,4>;
using Mat15 = Eigen::Matrix<matData_t, 1,5>;
using Mat16 = Eigen::Matrix<matData_t, 1,6>;
using Mat1X = Eigen::Matrix<matData_t, 1,Eigen::Dynamic>;


// Definition of templated-based fixed matrices using c'11 aliases
template<int D>
using Vect = Eigen::Matrix<matData_t, D,1, Eigen::RowMajor>;
template<int Rw,int Col>
using Mat = Eigen::Matrix<matData_t, Rw, Col, Eigen::RowMajor>;



}//end of namespace

#endif /* MATRIX_BASE_HPP_ */
