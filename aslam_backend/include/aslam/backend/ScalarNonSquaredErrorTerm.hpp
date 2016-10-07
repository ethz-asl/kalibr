#ifndef ASLAM_NON_SQUARED_ERROR_TERM_HPP
#define ASLAM_NON_SQUARED_ERROR_TERM_HPP

#include <sparse_block_matrix/sparse_block_matrix.h>
#include <boost/shared_ptr.hpp>
#include "backend.hpp"
#include "JacobianContainer.hpp"
#include "util/CommonDefinitions.hpp"
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
     * \class NonSquaredErrorTerm
     *
     * \brief a class representing a single scalar error term in a general
     * optimization problem of the form \f$ \sum_{i=0}^N w_i \cdot e_i \f$.
     */
    class ScalarNonSquaredErrorTerm {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      typedef boost::shared_ptr<aslam::backend::ScalarNonSquaredErrorTerm> Ptr;

      ScalarNonSquaredErrorTerm();
      virtual ~ScalarNonSquaredErrorTerm();

      /// \brief evaluate the error term and return the effective error.
      ///        This is equivalent to first call updateRawError() and then taking the result of getWeightedError();
      ///        After this is called, the _error is filled in with \f$ w \cdot e \f$
      double evaluateError() {
        updateRawError();
        return getWeightedError();
      }

      /// \brief update (compute and store) the raw error
      ///        After this is called, the _error is filled in with \f$ w \cdot e \f$
      ///        The raw error is returned.
      double updateRawError();

      /// \brief evaluate the Jacobians. Equivalent to evaluateWeightedJacobians.
      inline void evaluateJacobians(JacobianContainer & outJacobians) {
        Timer t("ScalarNonSquaredErrorTerm: evaluateJacobians", false);
        evaluateWeightedJacobians(outJacobians);
      }

      /// \brief evaluate the Jacobians using finite differences.
      void evaluateJacobiansFiniteDifference(JacobianContainer & outJacobians);
      
      /// \brief evaluate the raw Jacobians of \f$ w \cdot e \f$, not applying M-estimator policy and design variable scaling
      void evaluateRawJacobians(JacobianContainer & outJacobians);

      /// \brief evaluate the Jacobians of \f$ w \cdot e \f$ applying M-estimator policy and design variable scaling
      void evaluateWeightedJacobians(JacobianContainer& outJc);

      /// \brief Get the jacobians with or without M-estimator policy
      void evaluateJacobians(JacobianContainer& outJc, bool useMEstimator) {
        if(useMEstimator) evaluateWeightedJacobians(outJc);
        else evaluateRawJacobians(outJc);
      }

      /// \brief Get the error (before weighting by the M-estimator policy)
      double getRawError() const { return _error; }
      /// \brief Get the current, weighted error, i.e. with the M-estimator weight already applied.
      double getWeightedError() const { return getCurrentMEstimatorWeight() * getRawError(); }
      /// \brief Get the error with or without M-estimator policy
      inline double getError(bool useMEstimator) const {
        return useMEstimator ? getWeightedError() : getRawError();
      }

      /// \brief Getter for the weight
      inline double getWeight() const;

      /// \brief set the weight
      inline void setWeight(const double w);

      /// \brief returns a pointer to the MEstimator used. Return Null if the
      /// MEstimator used is not a MEstimatorType
      template <typename MEstimatorType>
      boost::shared_ptr<MEstimatorType> getMEstimatorPolicy();

      /// \brief set the M-Estimator policy. This function takes a squared error
      ///        and returns a weight to apply to that error term.
      void setMEstimatorPolicy(const boost::shared_ptr<MEstimator> & mEstimator);

      /// \brief clear the m-estimator policy.
      void clearMEstimatorPolicy();

      /// \brief compute the M-estimator weight from a squared error.
      double getMEstimatorWeight(double error) const { return _mEstimatorPolicy->getWeight(fabs(error)); }

      double getCurrentMEstimatorWeight() const { return getMEstimatorWeight(getRawError()); }

      /// \brief get the name of the M-Estimator.
      std::string getMEstimatorName();

      /// \brief How many design variables is this error term connected to?
      size_t numDesignVariables() const;

      /// \brief Get design variable i.
      DesignVariable* designVariable(size_t i);

      /// \brief Get design variable i.
      const DesignVariable* designVariable(size_t i) const;

      /// \brief Fill the set with all design variables.
      void getDesignVariables(DesignVariable::set_t& dvs);

      /// \brief Get the design variables
      const std::vector<DesignVariable*> & designVariables() const;

      void setTime(const sm::timing::NsecTime& t);
      sm::timing::NsecTime getTime() const { return _timestamp; }

      /// \brief Get the error term dimension. For compatibility with squared error term interface.
      inline size_t dimension() const { return 1UL; }

    protected:

      /// \brief evaluate the error term and return the scalar error \f$ e \f$
      virtual double evaluateErrorImplementation() = 0;

      /// \brief evaluate the Jacobians for \f$ e \f$
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) = 0;

      /// \brief child classes should set the set of design variables using this function.
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
      /// \brief the error \f$ w \cdot e \f$
      double _error;

      /// \brief the weight in \f$ w \cdot e \f$
      double _w;

      /// \brief The list of design variables.
      std::vector<DesignVariable*> _designVariables;

      sm::timing::NsecTime _timestamp;
    };


  } // namespace backend
} // namespace aslam

#include "implementation/ScalarNonSquaredErrorTerm.hpp"

#endif /* ASLAM_NON_SQUARED_ERROR_TERM_HPP */
