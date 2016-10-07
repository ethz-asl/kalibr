#ifndef ASLAM_BACKEND_COMPRESSED_COLUMN_JACOBIAN_TRANSPOSE_BUILDER_HPP
#define ASLAM_BACKEND_COMPRESSED_COLUMN_JACOBIAN_TRANSPOSE_BUILDER_HPP

#include "JacobianBuilder.hpp"
#include "CompressedColumnMatrix.hpp"

namespace aslam {
  namespace backend {

    /**
     * \class CompressedColumnJacobianTransposeBuilder
     *
     * Multithreaded code for building \f$ \mathbf J^T \f$ from the list of errors.
     */
    template<typename INDEX_T = int>
    class CompressedColumnJacobianTransposeBuilder {
    public:

      /// \brief the index type of the matrix
      typedef INDEX_T index_t;

      CompressedColumnJacobianTransposeBuilder();
      virtual ~CompressedColumnJacobianTransposeBuilder();

      /// \brief initialize the internal structure of the matrix.
      ///
      /// This function assumes that all the design variables in the DV container are active
      /// and that the container is sorted by order of block index such that dv[i]->blockIndex() == i
      ///
      virtual void initMatrixStructure(const std::vector<DesignVariable*>& dvs, const std::vector<ErrorTerm*>& errors);

      /// \brief build the large, sparse internal Jacobian matrix from the error terms.
      virtual void buildSystem(size_t nThreads, bool useMEstimator);

      /// \brief Get a view of the transpose of the Jacobian as a cholmod sparse matrix.
      virtual cholmod_sparse getJacobianTransposeView();

      /// \brief Get a const version of the compressed column matrix.
      CompressedColumnMatrix<index_t> & J_transpose();

      /// \brief Get a const version of the compressed column matrix.
        const CompressedColumnMatrix<index_t> & J_transpose() const;

    private:
      /// \brief a function to be run by a single thread.
      void evaluateJacobians(int threadId, int startIdx, int endIdx, bool useMEstimator);

      /// \brief The transpose of the Jacobian matrix has better cache coherency.
      CompressedColumnMatrix<index_t> _J_transpose;

      /// \brief The Jacobian, transposed, transposed.
      boost::shared_ptr<cholmod_sparse> _J;

      /// \brief is the structure initialized
      bool _isInitialized;

      struct Evaluator {
        void set(const JacobianColumnPointer& j, ErrorTerm* e, size_t er) {
          jcp = j;
          errorTerm = e;
          eRow = er;
        }
        JacobianColumnPointer jcp;
        size_t eRow;
        ErrorTerm* errorTerm;
      };

      /// \brief An array parallel to the error term array that maps error terms to parts of the Jacobian.
      std::vector<Evaluator> _jacobianPointers;

      /// \brief have we built the Jacobian from the transpose?
      bool _isJacobianBuiltFromJacobianTranspose;

      template<typename MEMBER_FUNCTION_PTR>
      void setupThreadedJob(MEMBER_FUNCTION_PTR ptr, size_t nThreads, bool useMEstimator);

    };
  } // namespace backend
} // namespace aslam


#include "implementation/CompressedColumnJacobianTransposeBuilder.hpp"



#endif /* ASLAM_BACKEND_COMPRESSED_COLUMN_JACOBIAN_TRANSPOSE_BUILDER_HPP */
