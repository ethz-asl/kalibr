#ifndef ASLAM_BACKEND_DESIGN_VARIABLE_UNIT_QUATERNION_HPP
#define ASLAM_BACKEND_DESIGN_VARIABLE_UNIT_QUATERNION_HPP

#include <aslam/backend/JacobianContainer.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include "DesignVariableGenericVector.hpp"
#include "QuaternionExpression.hpp"

namespace aslam {
namespace backend {
namespace quaternion {

template<typename Scalar_ = double, enum QuaternionMode Mode_ = DefaultQuaternionMode, enum UnitQuaternionGeometry Geometry_ = DefaultUnitQuaternionGeometry>
class DesignVariableUnitQuaternion : public DesignVariableGenericVector<4, Scalar_> {
 private:
  typedef internal::EigenQuaternionCalculator<Scalar_, Mode_> Calculator;
 public:
  typedef DesignVariableGenericVector<4, Scalar_> base_t;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Matrix<Scalar_, 4, 1> vector_t;

  DesignVariableUnitQuaternion(vector_t v = Calculator::getIdentity())
      : base_t(v) {
  }
  virtual ~DesignVariableUnitQuaternion() {
  }
 protected:

  /// \brief Update the design variable.
  virtual void updateImplementation(const double * dp, int size) {
    SM_ASSERT_EQ(std::runtime_error, size, 3, "update size must match the vector dimension.")
    this->_p_v = this->_currentValue;
    this->_currentValue = Calculator::template update<Geometry_>(this->_currentValue, Eigen::Map<const Eigen::Matrix<double, 3, 1> >(dp).template cast<Scalar_>());
  }
  /// \brief what is the number of dimensions of the perturbation variable.
  virtual int minimalDimensionsImplementation() const {
    return 3;
  }

  virtual void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const typename base_t::differential_t & diff) const {
    if(Geometry_ == UnitQuaternionGeometry::RIGHT_TRANSLATED){
      diff.addToJacobianContainer(outJacobians, (const DesignVariable *) this, Calculator::template dUpdate<Geometry_>(this->evaluate()));
    }
  }


};

}  // namespace quaternion
}  // namespace backend
}  // namespace aslam

#endif /* ASLAM_BACKEND_DESIGN_VARIABLE_UNIT_QUATERNION_HPP */
