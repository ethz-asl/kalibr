#ifndef ASLAM_BACKEND_DESIGN_VARIABLE_VECTOR_HPP
#define ASLAM_BACKEND_DESIGN_VARIABLE_VECTOR_HPP

#include <aslam/backend/JacobianContainer.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include "VectorExpressionNode.hpp"
#include "VectorExpression.hpp"

namespace aslam {
  namespace backend {
    template<int D>
    class DesignVariableVector : public DesignVariable, public VectorExpressionNode<D>
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef Eigen::Matrix<double, D, 1> vector_t;

        SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

      DesignVariableVector(vector_t v = vector_t::Zero());
      virtual ~DesignVariableVector();
      const vector_t & value() const;
      VectorExpression<D> toExpression();
    protected:
      /// \brief Revert the last state update.
      virtual void revertUpdateImplementation() override;
      /// \brief Update the design variable.
      virtual void updateImplementation(const double * dp, int size) override;
      /// \brief what is the number of dimensions of the perturbation variable.
      virtual int minimalDimensionsImplementation() const override;

      virtual vector_t evaluateImplementation() const override;
      virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const override;
      virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const override;

      /// Returns the content of the design variable
      virtual void getParametersImplementation(Eigen::MatrixXd& value) const override;

      /// Sets the content of the design variable
      virtual void setParametersImplementation(const Eigen::MatrixXd& value) override;

      /// Computes the minimal distance in tangent space between the current value of the DV and xHat
	  virtual void minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const override;

	  /// Computes the minimal distance in tangent space between the current value of the DV and xHat and the jacobian
	  virtual void minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const override;

    private:
      vector_t _v;
      vector_t _p_v;
      
    };


  } // namespace backend
} // namespace aslam



#include "implementation/DesignVariableVector.hpp"

#endif /* ASLAM_BACKEND_DESIGN_VARIABLE_VECTOR_HPP */
