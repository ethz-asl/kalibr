#include <aslam/GridCalibrationTargetDesignVariableContainer.hpp>

namespace aslam {

GridCalibrationTargetDesignVariableContainer::GridCalibrationTargetDesignVariableContainer(
    boost::shared_ptr<cameras::GridCalibrationTargetBase> target, bool active)
    : _target(target) {

  for (size_t i = 0; i < target->size(); ++i) {
    boost::shared_ptr<backend::MappedEuclideanPoint> pt(
        new backend::MappedEuclideanPoint(target->getPointDataPointer(i)));
    pt->setActive(active);
    _points.push_back(pt);
    _pointExpressions.push_back(pt->toExpression());
  }

}

bool GridCalibrationTargetDesignVariableContainer::isPointActive(size_t i) {
  SM_ASSERT_LT(std::runtime_error, i, _points.size(), "Out of bounds");
  return _points[i]->isActive();
}

GridCalibrationTargetDesignVariableContainer::~GridCalibrationTargetDesignVariableContainer() {

}

/// \brief get all underlying design variables.
void GridCalibrationTargetDesignVariableContainer::getDesignVariables(
    backend::DesignVariable::set_t & designVariables) const {
  for (size_t i = 0; i < _points.size(); ++i) {
    designVariables.insert(_points[i].get());
  }
}

/// \brief estimate the ith point.
void GridCalibrationTargetDesignVariableContainer::setPointActive(size_t i,
                                                                  bool active) {
  SM_ASSERT_LT(std::runtime_error, i, _points.size(), "Out of bounds");
  _points[i]->setActive(active);
}

/// \brief get the expression for point i
backend::EuclideanExpression GridCalibrationTargetDesignVariableContainer::getPoint(
    size_t i) {
  SM_ASSERT_LT(std::runtime_error, i, _points.size(), "Out of bounds");
  return _pointExpressions[i];
}

/// \brief get the target.
boost::shared_ptr<cameras::GridCalibrationTargetBase> GridCalibrationTargetDesignVariableContainer::getTarget() {
  return _target;
}

}  // namespace aslam
