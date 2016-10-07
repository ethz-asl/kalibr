#ifndef ASLAM_BACKEND_DV_ADAPTER_HPP
#define ASLAM_BACKEND_DV_ADAPTER_HPP

#include <aslam/backend/DesignVariable.hpp>
#include <boost/shared_ptr.hpp>
#include <sm/boost/null_deleter.hpp>

namespace aslam {
  namespace backend {

    template<typename T>
    class DesignVariableAdapter : public DesignVariable {
    public:

      // a variable wrapped with the DesignVariableAdapter has to define the
      // DesignVariableDimension enum (requirement of apriori error terms)
      //enum {
      //  DesignVariableDimension = T::DesignVariableDimension
      //};

      /// \brief The dv pointer must remain valid for the life of the optimization problem.
      DesignVariableAdapter(T* dv, bool adapterOwnsPointer);

      DesignVariableAdapter(const boost::shared_ptr<T>& dv);

      virtual ~DesignVariableAdapter();

      T& value();
      const T& value() const;

      Eigen::MatrixXd getParameters();

      boost::shared_ptr<T> valuePtr();
      boost::shared_ptr<const T> constValuePtr() const;


    private:
      /// \brief what is the number of dimensions of the perturbation variable.
      virtual int minimalDimensionsImplementation() const;

      /// \brief Update the design variable.
      virtual void updateImplementation(const double* dp, int size);

      /// \brief Revert the last state update.
      virtual void revertUpdateImplementation();

      /// Returns the content of the design variable
      virtual void getParametersImplementation(Eigen::MatrixXd& value) const;

      /// Sets the content of the design variable
      virtual void setParametersImplementation(const Eigen::MatrixXd& value);

      /// \brief the design variable being wrapped.
      boost::shared_ptr<T> _dv;

      /// \brief a backup for reverting the state.
      Eigen::MatrixXd _backup;

      virtual void minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const;

      virtual void minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const;

    };

  } // namespace backend
} // namespace aslam

#include "implementation/DesignVariableAdapter.hpp"

#endif /* ASLAM_BACKEND_DV_ADAPTER_HPP */
