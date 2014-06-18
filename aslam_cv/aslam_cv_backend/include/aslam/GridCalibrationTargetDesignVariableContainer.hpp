#ifndef ASLAM_GRID_CALIBRATION_TARGET_DESIGN_VARIABLE_CONTAINER_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_DESIGN_VARIABLE_CONTAINER_HPP

#include <boost/shared_ptr.hpp>
#include <aslam/targets.hpp>
#include <aslam/backend/MappedEuclideanPoint.hpp>

namespace aslam {

class GridCalibrationTargetDesignVariableContainer {
 public:
  GridCalibrationTargetDesignVariableContainer(
      boost::shared_ptr<cameras::GridCalibrationTargetBase> target,
      bool active);
  virtual ~GridCalibrationTargetDesignVariableContainer();

  /// \brief get all underlying design variables.
  void getDesignVariables(
      backend::DesignVariable::set_t & designVariables) const;

  /// \brief estimate the ith point.
  void setPointActive(size_t i, bool active);

  /// \brief is point i being estimated?
  bool isPointActive(size_t i);

  /// \brief get the expression for point i
  backend::EuclideanExpression getPoint(size_t i);

  /// \brief get the target.
  boost::shared_ptr<cameras::GridCalibrationTargetBase> getTarget();

 private:
  boost::shared_ptr<cameras::GridCalibrationTargetBase> _target;

  std::vector<boost::shared_ptr<backend::MappedEuclideanPoint> > _points;
  std::vector<backend::EuclideanExpression> _pointExpressions;

};

}  // namespace aslam

#endif /* ASLAM_GRID_CALIBRATION_TARGET_DESIGN_VARIABLE_CONTAINER_HPP */
