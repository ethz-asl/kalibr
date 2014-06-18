#ifndef ASLAM_GRID_CALIBRATION_TARGET_OBSERVATION_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_OBSERVATION_HPP

#include <vector>
#include <opencv2/core/core.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>
#include <sm/opencv/serialization.hpp>
#include <sm/eigen/serialization.hpp>
#include <sm/boost/serialization.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/assert_macros.hpp>
#include <aslam/Time.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras/GridCalibrationTargetBase.hpp>

namespace aslam {
namespace cameras {

/**
 * \class GridCalibrationTargetObservation
 * \brief an object representing the observation of a gridded calibration pattern in an image
 *
 */
class GridCalibrationTargetObservation {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  GridCalibrationTargetObservation(GridCalibrationTargetBase::Ptr target);
  // optionally store the image.
  GridCalibrationTargetObservation(GridCalibrationTargetBase::Ptr target, cv::Mat image);

  virtual ~GridCalibrationTargetObservation();

  /// \brief get a copy of the points
  Eigen::MatrixXd points() {
    return _points;
  };

  /// \brief set the image.
  void setImage(cv::Mat image);

  /// \brief clear the image. This can be handy for saving memory.
  void clearImage();

  /// \brief get all corners in image coordinates (order matches getCornersImageFrame)
  unsigned int getCornersTargetFrame(std::vector<cv::Point3f> &outCornerList) const;

  /// \brief get all corners in target frame coordinates (order matches getObservedTargetFrame)
  unsigned int getCornersImageFrame(std::vector<cv::Point2f> &outCornerList) const;

  /// \brief get the point index of all (observed) corners (order corresponds to the output
  ///        of getCornersImageFrame and getCornersTargetFrame)
  unsigned int getCornersIdx(std::vector<unsigned int> &outCornerIdx) const;

  /// \brief get the the reprojected corners of all observed corners
  unsigned int getCornerReprojection(const boost::shared_ptr<CameraGeometryBase> cameraGeometry,
                                     std::vector<cv::Point2f> &outPointReproj) const;

  /// \brief get a point from the target expressed in the target frame
  /// \return true if the grid point was seen in this image.
  bool imagePoint(size_t i, Eigen::Vector2d & outPoint) const;

  /// \brief update an image observation
  void updateImagePoint(size_t i, const Eigen::Vector2d & point);

  /// \brief remove an image observation
  void removeImagePoint(size_t i);

  /// \brief get a point from the target expressed in the target frame
  /// \return true if the grid point was seen in this image.
  bool imageGridPoint(size_t r, size_t c, Eigen::Vector2d & outPoint) const;

  /// \brief get the number of rows in the image
  size_t imRows() const {
    return _imRows;
  };

  /// \brief get the number of columns in the image.
  size_t imCols() const {
    return _imCols;
  };

  /// \brief get the image that generated these observations.
  cv::Mat image() const {
    return _image;
  };

  /// \brief get the grid calibration target
  GridCalibrationTargetBase::Ptr target() {
    return _target;
  };

  /// \brief get the grid calibration target
  GridCalibrationTargetBase::ConstPtr target() const {
    return _target;
  };

  void setTarget(GridCalibrationTargetBase::Ptr target);

  /// \brief get the transformation that takes points from
  ///        camera coordinates to target coordinates.
  const sm::kinematics::Transformation& T_t_c() const {
    return _T_t_c;
  };

  void set_T_t_c(const sm::kinematics::Transformation & T_t_c) {
    _T_t_c_isSet = true;
    _T_t_c = T_t_c;
  };

  /// \brief get the time of the observation
  aslam::Time time() const {
    return _stamp;
  };

  /// \brief set the time of the observation
  void setTime(const aslam::Time & stamp) {
    _stamp = stamp;
  };

  /// \brief return true if the class has at least one successful observation
  bool hasSuccessfulObservation() const;

 private:
  /// \brief the target being observed
  GridCalibrationTargetBase::Ptr _target;

  /// \brief the image observations stored row-major (idx = cols * r + c)
  Eigen::MatrixXd _points;

  /// \brief the success/failure of the image observations
  std::vector<bool> _success;

  /// \brief the transformation that takes points from
  ///        camera coordinates to target coordinates.
  sm::kinematics::Transformation _T_t_c;

  /// \brief the image that generated these observations
  cv::Mat _image;

  /// \brief the number of rows in the image (needed if the image is not stored)
  size_t _imRows;

  /// \brief the number of cols in the image (needed if the image is not stored)
  size_t _imCols;

  /// \brief the image timestamp
  aslam::Time _stamp;

  /// \brief has the extrinsics been set?
  bool _T_t_c_isSet;


  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
 public:
  enum {CLASS_SERIALIZATION_VERSION = 1};
  BOOST_SERIALIZATION_SPLIT_MEMBER()

  //serialization ctor
  GridCalibrationTargetObservation() {};

 protected:
  friend class boost::serialization::access;

  template<class Archive>
  void load(Archive & ar, const unsigned int version) {
    SM_ASSERT_LE(std::runtime_error, version,
                 (unsigned int)CLASS_SERIALIZATION_VERSION,
                 "Unsupported serialization version");
    ar >> BOOST_SERIALIZATION_NVP(_target);
    ar >> BOOST_SERIALIZATION_NVP(_points);
    ar >> BOOST_SERIALIZATION_NVP(_success);
    ar >> BOOST_SERIALIZATION_NVP(_T_t_c);
    ar >> BOOST_SERIALIZATION_NVP(_image);
    ar >> BOOST_SERIALIZATION_NVP(_imRows);
    ar >> BOOST_SERIALIZATION_NVP(_imCols);
    ar >> BOOST_SERIALIZATION_NVP(_stamp);
    ar >> BOOST_SERIALIZATION_NVP(_T_t_c_isSet);
  }

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    ar << BOOST_SERIALIZATION_NVP(_target);
    ar << BOOST_SERIALIZATION_NVP(_points);
    ar << BOOST_SERIALIZATION_NVP(_success);
    ar << BOOST_SERIALIZATION_NVP(_T_t_c);
    ar << BOOST_SERIALIZATION_NVP(_image);
    ar << BOOST_SERIALIZATION_NVP(_imRows);
    ar << BOOST_SERIALIZATION_NVP(_imCols);
    ar << BOOST_SERIALIZATION_NVP(_stamp);
    ar << BOOST_SERIALIZATION_NVP(_T_t_c_isSet);
  }
};

}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetObservation);

#endif /* ASLAM_GRID_CALIBRATION_TARGET_OBSERVATION_HPP */
