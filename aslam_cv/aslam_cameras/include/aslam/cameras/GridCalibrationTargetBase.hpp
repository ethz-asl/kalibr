#ifndef ASLAM_GRID_CALIBRATION_TARGET_BASE_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_BASE_HPP

#include <vector>
#include <utility>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <sm/eigen/serialization.hpp>
#include <sm/boost/serialization.hpp>
#include <sm/assert_macros.hpp>

namespace aslam {
namespace cameras {

/**
 * \class GridCalibrationTarget
 * \brief a class that represents a calibration target with known geometry
 *
 * The class is a little limiting:
 * The target is supposed to be square such that each row has the same number of points.
 * Points along a row are supposed to be nominally colinear.
 * Points along a column are supposed to be nominally colinear.
 * Missing points are not allowed.
 *
 */
class GridCalibrationTargetBase {
 public:
  /// \brief initialize base class
  GridCalibrationTargetBase(size_t rows, size_t cols);
  virtual ~GridCalibrationTargetBase() {
  };

 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  typedef boost::shared_ptr<GridCalibrationTargetBase> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetBase> ConstPtr;

  /// \brief get the number of points of the full grid
  size_t size() const {
    return _rows * _cols;
  };

  /// \brief the number of rows in the calibration target
  size_t rows() const {
    return _rows;
  };

  /// \brief the number of columns in the calibration target
  size_t cols() const {
    return _cols;
  };

  /// \brief get a point from the target expressed in the target frame
  Eigen::Vector3d point(size_t i) const;

  /// \brief get all points from the target expressed in the target frame
  Eigen::MatrixXd points() const;

  /// \brief get the grid coordinates for a point
  std::pair<size_t, size_t> pointToGridCoordinates(size_t i) const;

  /// \brief get the point index from the grid coordinates
  size_t gridCoordinatesToPoint(size_t r, size_t c) const;

  /// \brief get a point from the target expressed in the target frame
  ///        by row and column
  Eigen::Vector3d gridPoint(size_t r, size_t c) const;

  /// \brief extract the calibration target points from an image
  ///        outCornerObserved flags wheter the corresponding point
  ///        in outImagePoints was observed
  virtual bool computeObservation(const cv::Mat & /*image*/,
                                  Eigen::MatrixXd & /*outImagePoints*/,
                                  std::vector<bool> & /*outCornerObserved*/) const
  {
    SM_ASSERT_TRUE(Exception, true, "you need to implement this method for each target!");
    return false;
  };

  /// \brief return pointer to the i-th grid point in target frame
  double * getPointDataPointer(size_t i);

 protected:
  /// \brief grid points stored in row-major order (idx = cols * r + c)
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> _points;

  /// \brief the number of point rows in the calibration target
  size_t _rows;

  /// \brief the number of point columns in the calibration target
  size_t _cols;

  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
 public:
  enum {CLASS_SERIALIZATION_VERSION = 1};
  BOOST_SERIALIZATION_SPLIT_MEMBER()

  //serialization ctor
  GridCalibrationTargetBase() {};

 protected:
  friend class boost::serialization::access;

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    ar << BOOST_SERIALIZATION_NVP(_points);
    ar << BOOST_SERIALIZATION_NVP(_rows);
    ar << BOOST_SERIALIZATION_NVP(_cols);
  }
  template<class Archive>
  void load(Archive & ar, const unsigned int /* version */) {
    ar >> BOOST_SERIALIZATION_NVP(_points);
    ar >> BOOST_SERIALIZATION_NVP(_rows);
    ar >> BOOST_SERIALIZATION_NVP(_cols);
  }

}; //class GridCalibrationTargetBase

} // namespace cameras
} // namespace aslam

SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetBase);
BOOST_CLASS_EXPORT_KEY(aslam::cameras::GridCalibrationTargetBase);

#endif /* ASLAM_GRID_CALIBRATION_TARGET_BASE_HPP */
