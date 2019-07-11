/* Copyright 2018-2019 Skolkovo Institute of Science and Technology (Skoltech)
 * All rights reserved.
 *
 * CustomCholesky.hpp
 *
 *  Created on: Jan 27, 2019
 *      Author: Konstantin Pakulev
 *              konstantin.pakulev@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */

#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

using namespace Eigen;

#ifndef MROB_CUSTOMCHOLESKY_H
#define MROB_CUSTOMCHOLESKY_H

namespace Eigen {

    template<typename _MatrixType, int _UpLo = Lower, typename _Ordering = NaturalOrdering<int>> class CustomCholesky;

    namespace internal {
        template<typename _MatrixType, int _UpLo, typename _Ordering>
        struct traits<CustomCholesky<_MatrixType, _UpLo, _Ordering> > {
            typedef _MatrixType MatrixType;
            typedef _Ordering OrderingType;
            enum { UpLo = _UpLo };
            typedef typename MatrixType::Scalar                         Scalar;
            typedef typename MatrixType::StorageIndex                   StorageIndex;
            typedef SparseMatrix<Scalar, ColMajor, StorageIndex>        CholMatrixType;
            typedef TriangularView<const CholMatrixType, Eigen::Lower>  MatrixL;
            typedef TriangularView<const typename CholMatrixType::AdjointReturnType, Eigen::Upper>   MatrixU;
            static inline MatrixL getL(const MatrixType& m) { return MatrixL(m); }
            static inline MatrixU getU(const MatrixType& m) { return MatrixU(m.adjoint()); }
        };
    }


    template<typename _MatrixType, int _UpLo, typename _Ordering>
    class CustomCholesky : public Eigen::SimplicialCholeskyBase<CustomCholesky<_MatrixType, _UpLo, _Ordering> > {
    public:
        typedef _MatrixType MatrixType;
        enum { UpLo = _UpLo };
        typedef SimplicialCholeskyBase<CustomCholesky> Base;
        typedef typename MatrixType::Scalar Scalar;
        typedef internal::traits<CustomCholesky> Traits;
        typedef typename Traits::MatrixL  MatrixL;
        typedef typename Traits::MatrixU  MatrixU;
    public:
        CustomCholesky() : Base() {}

        explicit CustomCholesky(const _MatrixType &matrix)
                : Base(matrix) {}


        CustomCholesky &compute(const _MatrixType &matrix) {
            Base::template compute<false>(matrix);
            return *this;
        }

        void analyzePattern(const _MatrixType &a) {
            Base::analyzePattern(a, false);
        }

        void factorize(const _MatrixType &a) {
            Base::template factorize<false>(a);
        }


        inline const MatrixL matrixL() const {
            eigen_assert(Base::m_factorizationIsOk && "Simplicial LLT not factorized");
            return Traits::getL(Base::m_matrix);
        }

        inline const MatrixU matrixU() const {
            eigen_assert(Base::m_factorizationIsOk && "Simplicial LLT not factorized");
            return Traits::getU(Base::m_matrix);
        }

        MatrixType& getL() {
            return Base::m_matrix;
        }
    };
}


#endif //MROB_CUSTOMCHOLESKY_H
