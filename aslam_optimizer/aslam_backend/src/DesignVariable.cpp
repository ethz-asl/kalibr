#include <aslam/backend/DesignVariable.hpp>

namespace aslam {
  namespace backend {

    DesignVariable::DesignVariable() :
      _blockIndex(-1), _columnBase(-1), _isMarginalized(false), _isActive(false), _scaling(1.0)
    {
    }


    DesignVariable::~DesignVariable()
    {
    }


    /// \brief update the design variable.
    void DesignVariable::update(const double* dp, int size)
    {
      // scale the design variable:
      updateImplementation(dp, size);
    }


    /// \brief Revert the last state update
    void DesignVariable::revertUpdate()
    {
      revertUpdateImplementation();
    }


    /// \brief is this design variable active in the optimization.
    bool DesignVariable::isActive() const
    {
      return _isActive;
    }


    /// \brief set the active state of this design variable.
    void DesignVariable::setActive(bool active)
    {
      _isActive = active;
    }


    /// \brief should this variable be marginalized in the schur-complement step?
    bool DesignVariable::isMarginalized() const
    {
      return _isMarginalized;
    }


    /// \brief should this variable be marginalized in the schur-complement step?
    void DesignVariable::setMarginalized(bool marginalized)
    {
      _isMarginalized = marginalized;
    }



    /// \brief get the block index used in the optimization routine. -1 if not being optimized.
    int DesignVariable::blockIndex() const
    {
      return _blockIndex;
    }


    /// \brief set the block index used in the optimization routine.
    void DesignVariable::setBlockIndex(int blockIndex)
    {
      _blockIndex = blockIndex;
    }

    /// \brief what is the number of dimensions of the perturbation variable.
    int DesignVariable::minimalDimensions() const
    {
      return minimalDimensionsImplementation();
    }

    /// \brief set the scaling of this design variable used in the optimizaiton.
    void DesignVariable::setScaling(double scaling)
    {
      _scaling = scaling;
    }

    /// \brief get the scaling of this design variable used in the optimizaiton.
    double DesignVariable::scaling() const
    {
      return _scaling;
    }

    /// \brief The column base of this block in the Jacobian matrix
    int DesignVariable::columnBase() const
    {
      return _columnBase;
    }

    /// \brief Set the column base of this block in the Jacobian matrix
    void DesignVariable::setColumnBase(int columnBase)
    {
      _columnBase = columnBase;
    }


    void DesignVariable::getParameters(Eigen::MatrixXd& value) const {
      getParametersImplementation(value);
    }

    void DesignVariable::setParameters(const Eigen::MatrixXd& value) {
      setParametersImplementation(value);
    }

    /// \brief Computes the minimal distance in tangent space between the current value of the DV and xHat
    void DesignVariable::minimalDifference(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference) const
    {
    	minimalDifferenceImplementation(xHat, outDifference);
    }
    /// Computes the minimal distance in tangent space between the current value of the DV and xHat and the jacobian
    void DesignVariable::minimalDifferenceAndJacobian(const Eigen::MatrixXd& xHat, Eigen::VectorXd& outDifference, Eigen::MatrixXd& outJacobian) const
    {
    	minimalDifferenceAndJacobianImplementation(xHat, outDifference, outJacobian);
    }



  void DesignVariable::minimalDifferenceImplementation(const Eigen::MatrixXd& /*xHat*/, Eigen::VectorXd& /*outDifference*/ ) const
    {
    	SM_THROW(aslam::Exception, "Calling default dummy implementation of minimalDifference(). If you want to use the marginalizer with this design variable, implement a specialization of this function first!");
    }

  void DesignVariable::minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& /*xHat*/, Eigen::VectorXd& /*outDifference*/, Eigen::MatrixXd& /*outJacobian*/ ) const
    {
    	SM_THROW(aslam::Exception, "Calling default dummy implementation of minimalDifferenceAndJacobian(). If you want to use the marginalizer with this design variable, implement a specialization of this function first!");

    }

  } // namespace backend
} // namespace aslam

