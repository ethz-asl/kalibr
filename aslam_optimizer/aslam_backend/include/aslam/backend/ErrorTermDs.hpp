#ifndef ASLAM_ERROR_TERM_DS_HPP
#define ASLAM_ERROR_TERM_DS_HPP

#include <sparse_block_matrix/sparse_block_matrix.h>
#include <boost/shared_ptr.hpp>
#include <aslam/Exceptions.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <sm/timing/Timer.hpp>
#include <sm/eigen/matrix_sqrt.hpp>
#include <aslam/backend/ErrorTerm.hpp>

namespace aslam {
  namespace backend {
    /**
     * \class ErrorTermDs
     * \brief An implementation of a vector-valued error term.
     *
     * This class fills in some of the functionality needed for "normal", vector-valued error terms.
     * In most cases, one will want to derive from this class when implementing an error term.
     *
     */
    class ErrorTermDs : public aslam::backend::ErrorTerm {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW


      typedef Eigen::VectorXd error_t;
      typedef Eigen::MatrixXd inverse_covariance_t;

      ErrorTermDs(int dimensionErrorTerm);
      virtual ~ErrorTermDs();

      /// \brief retrieve the error vector
      const error_t& error() const;

      /// \brief Get the square root of the inverse covariance matrix.
      /// To recover the inverse covariance matrix from this matrix,
      /// \f$ A \f$, use \f$ \mathbf A \mathbf A^T\f$
      const inverse_covariance_t& sqrtInvR() const;

      /// \brief the inverse covariance matrix.
      inverse_covariance_t invR() const;

      virtual void getInvR(Eigen::MatrixXd& invR) const;
      virtual Eigen::MatrixXd vsInvR() const;
      virtual void vsSetInvR(const Eigen::MatrixXd& invR);

      virtual void getWeightedJacobians(JacobianContainer& outJc, bool useMEstimator);
      virtual void getWeightedError(Eigen::VectorXd& e, bool useMEstimator) const;

      /// Check if Jacobians are finite
      void checkJacobiansFinite() const;
      /// Check if analytical and numerical Jacobians match
      void checkJacobiansNumerical(double tolerance = 1e-6);

    protected:

      /// \brief build the hessian.
      virtual void buildHessianImplementation(SparseBlockMatrix& outHessian, Eigen::VectorXd& outRhs, bool useMEstimator);

      /// \brief Clear the Jacobians
      virtual void clearJacobians();

      /// \brief get the current value of the error.
      Eigen::VectorXd vsErrorImplementation() const;

      virtual size_t getDimensionImplementation() const {
        return _dimensionErrorTerm;
      }

      // This probably has to have a jacobian container...
      // \todo make this fixed sized.
      JacobianContainer _jacobians;

      /// \brief set the error vector.
      //void setError(const Eigen::MatrixBase<double> & e);
      void setError(const Eigen::VectorXd& e);

      /// \brief set the inverse covariance matrix. Note, this will compute the
      ///  square root of this matrix numerically. If you have error terms that
      ///  reuse the same uncertainty, consider computing the square root once and
      ///  calling setSqrtInvR()
      void setInvR(const Eigen::MatrixXd& invR);

      /// \brief sets the square root inverse covariance matrix.
      void setSqrtInvR(const Eigen::MatrixXd& sqrtInvR);

      /// \brief evaluate the squared error from the error vector and
      ///        square root covariance matrix.
      double evaluateChiSquaredError() const;
      
      /// Initializes the jacobian container with a new number of rows
      void resizeJacobianContainer(int nrows);

    protected:
      int _dimensionErrorTerm;
      //int _dimensionDesignVariables;

    private:
      /// \brief the error term. This must be filled in on a call to evaluateError()

      error_t _error;
      /// \brief the inverse uncertainty matrix.
      inverse_covariance_t _sqrtInvR;

      // swap to enable
      //typedef sm::timing::Timer Timer;
      // swap to disable
      typedef sm::timing::DummyTimer Timer;

      Timer _evalJacobianTimer;
      Timer _buildHessianTimer;
    };

  } // namespace backend
} // namespace aslam

#endif /* ASLAM_ERROR_TERM_DS_HPP */
