#ifndef ASLAM_SCALAR_EXPRESSION_NODE_KEYPOINT_TIME_HPP
#define ASLAM_SCALAR_EXPRESSION_NODE_KEYPOINT_TIME_HPP

#include <aslam/backend/ScalarExpressionNode.hpp>
#include <aslam/Time.hpp>
#include <aslam/Duration.hpp>

// forward definition of the camera design variable
namespace aslam {
  namespace backend {
    template<typename CAMERA_T>
    class CameraDesignVariable;
  }
}

namespace aslam {
  namespace backend {

    template<typename CAMERA_T>
    class ScalarExpressionNodeKeypointTime :
        public aslam::backend::ScalarExpressionNode
    {

      typedef CAMERA_T camera_t;

     public:
      ScalarExpressionNodeKeypointTime(
        const aslam::Time & stamp,
        const Eigen::VectorXd & y,
        boost::shared_ptr<backend::CameraDesignVariable<CAMERA_T> > dv
      );
      virtual ~ScalarExpressionNodeKeypointTime();

      virtual double toScalarImplementation() const;
      virtual void evaluateJacobiansImplementation(backend::JacobianContainer & outJacobians) const;
      virtual void evaluateJacobiansImplementation(
        backend::JacobianContainer & outJacobians,
        const Eigen::MatrixXd & applyChainRule
      ) const;
      virtual void getDesignVariablesImplementation(
        backend::DesignVariable::set_t & designVariables
      ) const;

     private:
      aslam::Time _stamp;
      Eigen::VectorXd _y;
      boost::shared_ptr<backend::CameraDesignVariable<CAMERA_T> > _dv;
    };

  } // namespace backend
}  // namespace aslam

#include "implementation/ScalarExpressionNodeKeypointTime.hpp"

#endif /* ASLAM_SCALAR_EXPRESSION_NODE_KEYPOINT_TIME_HPP */
