#ifndef ASLAM_BACKEND_HPP
#define ASLAM_BACKEND_HPP

#include <sm/assert_macros.hpp>
#include "DesignVariable.hpp"
#include <aslam/Exceptions.hpp>
#include <sparse_block_matrix/sparse_block_matrix.h>

namespace aslam {
  namespace backend {
    class DesignVariable;
    typedef sparse_block_matrix::SparseBlockMatrix<Eigen::MatrixXd> SparseBlockMatrix;

    struct SolutionReturnValue {
      SolutionReturnValue() :
        JStart(0.0), JFinal(0.0), iterations(0), failedIterations(0), lmLambdaFinal(0.0),
        dXFinal(0.0), dJFinal(0.0), linearSolverFailure(false) {}

      double JStart;
      double JFinal;
      int iterations;
      int failedIterations;
      double lmLambdaFinal;
      double dXFinal;
      double dJFinal;
      bool linearSolverFailure;
    };


  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_HPP */
