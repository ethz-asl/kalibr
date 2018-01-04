#ifndef ASLAM_ERROR_TERM_HPP
#define ASLAM_ERROR_TERM_HPP

#include <sparse_block_matrix/sparse_block_matrix.h>
#include <boost/shared_ptr.hpp>
#include "backend.hpp"
#include "JacobianContainer.hpp"
#include <aslam/Exceptions.hpp>
#include <sm/eigen/NumericalDiff.hpp>
#include <sm/timing/Timer.hpp>
#include "MEstimatorPolicies.hpp"
#include <sm/eigen/matrix_sqrt.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>

namespace aslam {
  namespace backend {
    class MEstimator;



    /**
     * \class ErrorTerm
     *
     * \brief a class representing a single error term in a nonlinear least squares problem.
     *
     * The ErrorTerm class is an abstract base class that is able to handle both "normal"
     * vector valued error terms, as one would find in a nonlinear least squares problem,
     * and "quadratic terms", the smoothing matrices found in continuous-time estimation problems.
     *
     * For normal error terms of dimension D, child classes should derive from ErrorTermFS<D>.
     */
    class ErrorTerm {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef boost::shared_ptr<aslam::backend::ErrorTerm> Ptr;

      ErrorTerm();
      virtual ~ErrorTerm();

      /// \brief evaluate the error term.
      ///        After this is called, the _squaredError is filled in with \f$ \mathbf e^T \mathbf R^{-1} \mathbf e \f$
      double evaluateError();

      /// \brief evaluate the Jacobians.
      void evaluateJacobians(JacobianContainer & outJacobians) const;

      /// \brief evaluate the Jacobians using finite differences.
      void evaluateJacobiansFiniteDifference(JacobianContainer & outJacobians);
      
      virtual void getWeightedJacobians(JacobianContainer& outJc, bool useMEstimator) = 0;

      virtual void getWeightedError(Eigen::VectorXd& e, bool useMEstimator) const = 0;

      /// \brief get the current value of the error.
      /// This was put here to make the python interface easier to generate. It doesn't
      /// fit for quadratic integral terms so it may go away in future versions.
      Eigen::VectorXd vsError() const;

      virtual void getInvR(Eigen::MatrixXd& invR) const = 0;
      virtual Eigen::MatrixXd vsInvR() const = 0;
      virtual void vsSetInvR(const Eigen::MatrixXd& invR) = 0;

      /// \brief set the M-Estimator policy. This function takes a squared error
      ///        and returns a weight to apply to that error term.
      void setMEstimatorPolicy(const boost::shared_ptr<MEstimator> & mEstimator);

      /// \brief clear the m-estimator policy.
      void clearMEstimatorPolicy();

      /// \brief compute the M-estimator weight from a squared error.
      double getMEstimatorWeight(double squaredError) const;

      double getCurrentMEstimatorWeight() const;

      /// \brief get the name of the M-Estimator.
      std::string getMEstimatorName();

      /// \brief build this error term's part of the Hessian matrix.
      ///
      /// the i/o variables outHessian and outRhs are the full Hessian and rhs in the Gauss-Newton
      /// problem. The correct blocks for each design varible are available from the design
      /// variable as dv.blockIndex()
      void buildHessian(SparseBlockMatrix& outHessian, Eigen::VectorXd& outRhs, bool useMEstimator) ;

      /// \brief How many design variables is this error term connected to?
      size_t numDesignVariables() const;

      /// \brief Get design variable i.
      DesignVariable* designVariable(size_t i);

      /// \brief Get design variable i.
      const DesignVariable* designVariable(size_t i) const;

      /// \brief Get the squared error (weighted by the M-estimator policy)
      double getWeightedSquaredError() const;

      // \brief Get the squared error (before weighting by the M-estimator policy)
      double getRawSquaredError() const;


      double getSquaredError(bool useMEstimator)const {
        return useMEstimator ? getWeightedSquaredError() : getRawSquaredError();
      }

      /// \brief Fill the set with all design variables.
      void getDesignVariables(DesignVariable::set_t& dvs);

      /// \breif Get the error term dimension
      size_t dimension() const;

      /// \brief Get the design variables
      const std::vector<DesignVariable*> & designVariables() const;

      /// \brief Get the column base of this error term in the Jacobian matrix.
      size_t rowBase() const;

      /// \brief Set the column base of this error term in the Jacobian matrix.
      void setRowBase(size_t);

      void setTime(const sm::timing::NsecTime& t);
      sm::timing::NsecTime getTime() { return _timestamp; }

    protected:

      /// \brief evaluate the error term and return the weighted squared error e^T invR e
      virtual double evaluateErrorImplementation() = 0;

      /// \brief evaluate the Jacobians
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const = 0;

      /// \brief get the number of dimensions of this error term.
      virtual size_t getDimensionImplementation() const = 0;

      /// \brief build this error term's part of the Hessian matrix.
      ///
      /// the i/o variables outHessian and outRhs are the full Hessian and rhs in the Gauss-Newton
      /// problem. The correct blocks for each design varible are available from the design
      /// variable as dv.blockIndex()
      virtual void buildHessianImplementation(SparseBlockMatrix& outHessian, Eigen::VectorXd& outRhs, bool useMEstimator) = 0;

      virtual Eigen::VectorXd vsErrorImplementation() const = 0;

      /// \brief child classes should set the set of design variables usign this function.
      void setDesignVariables(const std::vector<DesignVariable*> & designVariables);

      void setDesignVariables(DesignVariable* dv1);
      void setDesignVariables(DesignVariable* dv1, DesignVariable* dv2);
      void setDesignVariables(DesignVariable* dv1, DesignVariable* dv2, DesignVariable* dv3);
      void setDesignVariables(DesignVariable* dv1, DesignVariable* dv2, DesignVariable* dv3, DesignVariable* dv4);


      template<typename ITERATOR_T>
      void setDesignVariablesIterator(ITERATOR_T start, ITERATOR_T end);


      /// \brief the MEstimator policy for this error term
      boost::shared_ptr<MEstimator> _mEstimatorPolicy;

    private:
      /// \brief the squared error \f$ \mathbf e^T \mathbf R^{-1} \mathbf e \f$
      double _squaredError;

      /// \brief The list of design variables.
      std::vector<DesignVariable*> _designVariables;

      size_t _rowBase;

      sm::timing::NsecTime _timestamp;
    };



    /**
     * \class ErrorTermFs
     * \brief An implementation of a vector-valued error term.
     *
     * This class fills in some of the functionality needed for "normal", vector-valued error terms.
     * In most cases, one will want to derive from this class when implementing an error term.
     *
     */
    template<int DIMENSION>
    class ErrorTermFs : public ErrorTerm {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      enum {
        Dimension = DIMENSION
      };
      typedef Eigen::Matrix<double, Dimension, 1> error_t;
      typedef Eigen::Matrix<double, Dimension, Dimension> inverse_covariance_t;

      ErrorTermFs();
      virtual ~ErrorTermFs();


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

      /// \brief get the current value of the error.
      virtual Eigen::VectorXd vsErrorImplementation() const;

      virtual size_t getDimensionImplementation() const {
        return Dimension;
      }

      /// \brief set the error vector.
      template<typename DERIVED>
      void setError(const Eigen::MatrixBase<DERIVED> & e);


      /// \brief set the inverse covariance matrix. Note, this will compute the
      ///  square root of this matrix numerically. If you have error terms that
      ///  reuse the same uncertainty, consider computing the square root once and
      ///  calling setSqrtInvR()
      template<typename DERIVED>
      void setInvR(const Eigen::MatrixBase<DERIVED> & invR);

      /// \brief sets the square root inverse covariance matrix.
      template<typename DERIVED>
      void setSqrtInvR(const Eigen::MatrixBase<DERIVED> & sqrtInvR);

      /// \brief evaluate the squared error from the error vector and
      ///        square root covariance matrix.
      double evaluateChiSquaredError() const;

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

#include "implementation/ErrorTerm.hpp"

#endif /* ASLAM_ERROR_TERM_HPP */
