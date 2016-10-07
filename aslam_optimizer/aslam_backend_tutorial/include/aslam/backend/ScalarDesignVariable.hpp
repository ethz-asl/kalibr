#ifndef ASLAM_BACKEND_SCALAR_DESIGN_VARIABLE
#define ASLAM_BACKEND_SCALAR_DESIGN_VARIABLE

#include <aslam/backend/DesignVariable.hpp>
namespace aslam {
  namespace backend {
    
    class ScalarDesignVariable : public DesignVariable
    {
    public:
      /// \brief initialize the variable with some initial guess
      ScalarDesignVariable(double initialValue);
      virtual ~ScalarDesignVariable();
      
      /// \brief get the current value. This is not required but it will be used
      // by error terms to get the current value of the design variable.
      double value();
    protected:
      // The following three virtual functions are required by DesignVariable

      /// \brief what is the number of dimensions of the perturbation variable.
      virtual int minimalDimensionsImplementation() const;
      
      /// \brief Update the design variable.
      virtual void updateImplementation(const double * dp, int size);
      
      /// \brief Revert the last state update.
      virtual void revertUpdateImplementation();

      /// Returns the content of the design variable
      virtual void getParametersImplementation(Eigen::MatrixXd& value) const;

      /// Sets the content of the design variable
      virtual void setParametersImplementation(const Eigen::MatrixXd& value);

      /// Computes the minimal distance in tangent space between the current value of the DV and xHat
      virtual void minimalDifferenceImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const;

      /// Computes the minimal distance in tangent space between the current value of the DV and xHat and the jacobian
      virtual void minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const;


    private:
      /// \brief the value of the design variable
      double _value;
      /// \brief the previous version of the design variable
      double _p_value;
    };

  } // namespace backend
} // namespace aslam

#endif /* ASLAM_BACKEND_SCALAR_DESIGN_VARIABLE */
