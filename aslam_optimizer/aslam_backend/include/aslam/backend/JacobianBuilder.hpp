#ifndef ASLAM_BACKEND_COMPRESSED_ROW_JACOBIAN_BUILDER_HPP
#define ASLAM_BACKEND_COMPRESSED_ROW_JACOBIAN_BUILDER_HPP

#include <cholmod.h>
#include "ErrorTerm.hpp"

namespace aslam {
  namespace backend {

    class JacobianBuilder {
    public:
      JacobianBuilder();
      virtual ~JacobianBuilder();

      /// \brief initialize the internal structure of the matrix.
      ///
      /// This function assumes that all the design variables in the DV array are active
      /// and that the container is sorted by order of block index such that dv[i]->blockIndex() == i
      /// and that the column base is filled in.
      ///
      virtual void initMatrixStructure(const std::vector<DesignVariable*>& dvs, const std::vector<ErrorTerm*>& errors) = 0;

      /// \brief Evaluate the Chi Squared error.
      virtual double evaluateError(size_t nThreads, bool useMEstimator) = 0;

      /// \brief build the large, sparse internal Jacobian matrix from the error terms.
      virtual void buildJacobian(size_t nThreads, bool useMEstimator) = 0;

      /// \brief Get a view of the Jacobian as a cholmod sparse matrix.
      virtual cholmod_sparse getJacobianView() = 0;

      /// \brief Get a view of the transpose of the Jacobian as a cholmod sparse matrix.
      virtual cholmod_sparse getJacobianTransposeView() = 0;


    };

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_COMPRESSED_ROW_JACOBIAN_BUILDER_HPP */
