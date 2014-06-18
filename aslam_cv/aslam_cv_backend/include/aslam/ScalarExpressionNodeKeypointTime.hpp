#ifndef ASLAM_SCALAR_EXPRESSION_NODE_KEYPOINT_TIME_HPP
#define ASLAM_SCALAR_EXPRESSION_NODE_KEYPOINT_TIME_HPP

#include <aslam/backend/ScalarExpressionNode.hpp>
#include <aslam/CameraGeometryDesignVariableContainer.hpp>
namespace aslam {

class ScalarExpressionNodeKeypointTime :
    public aslam::backend::ScalarExpressionNode {
 public:
  ScalarExpressionNodeKeypointTime(
      const aslam::Time & stamp,
      const Eigen::VectorXd & y,
      boost::shared_ptr<
          backend::DesignVariableAdapter<CameraGeometryDesignVariableContainer> > dv);
  virtual ~ScalarExpressionNodeKeypointTime();

  virtual double toScalarImplementation() const;
  virtual void evaluateJacobiansImplementation(
      backend::JacobianContainer & outJacobians) const;
  virtual void evaluateJacobiansImplementation(
      backend::JacobianContainer & outJacobians,
      const Eigen::MatrixXd & applyChainRule) const;
  virtual void getDesignVariablesImplementation(
      backend::DesignVariable::set_t & designVariables) const;

 private:
  aslam::Time _stamp;
  Eigen::VectorXd _y;
  boost::shared_ptr<
      backend::DesignVariableAdapter<CameraGeometryDesignVariableContainer> > _dv;
};

}  // namespace aslam

#endif /* ASLAM_SCALAR_EXPRESSION_NODE_KEYPOINT_TIME_HPP */
