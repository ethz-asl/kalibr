#include <utility>
#include <aslam/cameras/GridCalibrationTargetBase.hpp>

namespace aslam {
namespace cameras {

GridCalibrationTargetBase::GridCalibrationTargetBase(size_t rows, size_t cols)
    : _rows(rows),
      _cols(cols) {
}

/// \brief get all points from the target expressed in the target frame
Eigen::MatrixXd GridCalibrationTargetBase::points() const {
  return _points;
}

/// \brief get a point from the target expressed in the target frame
Eigen::Vector3d GridCalibrationTargetBase::point(size_t i) const {
  return _points.row(i);
}

/// \brief get the grid coordinates for a point
std::pair<size_t, size_t> GridCalibrationTargetBase::pointToGridCoordinates(size_t i) const {
  return std::pair<size_t, size_t>(i % cols(), (int) i / cols());
}

/// \brief get the point index from the grid coordinates
size_t GridCalibrationTargetBase::gridCoordinatesToPoint(size_t r, size_t c) const {
  return cols() * r + c;
}

/// \brief get a point from the target expressed in the target frame
///        by row and column
Eigen::Vector3d GridCalibrationTargetBase::gridPoint(size_t r, size_t c) const {
  return _points.row(gridCoordinatesToPoint(r, c));
}

double * GridCalibrationTargetBase::getPointDataPointer(size_t i) {
  SM_ASSERT_LT(Exception, size()-1, i, "Grid has less points than the requested idx!")
  return &_points(i, 0);
}

}  // namespace cameras
}  // namespace aslam

//export explicit instantions for all included archives
#include <sm/boost/serialization.hpp>
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::GridCalibrationTargetBase);

