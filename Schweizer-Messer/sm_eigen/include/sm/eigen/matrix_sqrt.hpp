#ifndef SM_EIGEN_MATRIX_SQRT_HPP
#define SM_EIGEN_MATRIX_SQRT_HPP

#include <sm/assert_macros.hpp>
#include <Eigen/Cholesky>

namespace sm {
    namespace eigen {
        
        /** 
         * \brief Compute the square root of a matrix using the LDLt decomposition for square, positive semidefinite matrices
         *
         * To reconstruct the input matrix, \f$ \mathbf A \f$, from the returned matrix, \f$ \mathbf S \f$,
         * use, \f$ \mathbf A = \mathbf S \mathbf S^T \f$.
         * 
         * 
         * @param inMatrix      The square matrix whose square root should be computed.
         * @param outMatrixSqrt The output square root.
         */
        template<typename DERIVED1, typename DERIVED2>
        /*Eigen::ComputationInfo*/ void computeMatrixSqrt(const Eigen::MatrixBase<DERIVED1> & inMatrix,
                                                          const Eigen::MatrixBase<DERIVED2> & outMatrixSqrt)
        {
            SM_ASSERT_EQ_DBG(std::runtime_error, inMatrix.rows(), inMatrix.cols(), "This method is only valid for square input matrices");
            
            DERIVED2 & result = const_cast<DERIVED2 &>(outMatrixSqrt.derived());

            // This is tricky. Using the output matrix type causes the input matrix
            // type to be upgraded to a real numeric matrix. This is useful if, 
            // for example, the inMatrix is something like Eigen::Matrix3d::Identity(),
            // which is not an actual matrix. Using DERIVED1 as the template argument
            // in that case will cause a firestorm of compiler errors.
            Eigen::LDLT< DERIVED2 > ldlt(inMatrix.derived());
            result = ldlt.matrixL();
            result = ldlt.transpositionsP().transpose() * result;
            result *= ldlt.vectorD().array().sqrt().matrix().asDiagonal();
            
            //return ldlt.info();
            
        }

    } // namespace eigen
} // namespace sm

#endif /* SM_EIGEN_MATRIX_SQRT_HPP */
