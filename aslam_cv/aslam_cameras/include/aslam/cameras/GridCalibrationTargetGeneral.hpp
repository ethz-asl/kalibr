#ifndef ASLAM_GRID_CALIBRATION_TARGET_GENERAL_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_GENERAL_HPP

#include <vector>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/serialization/export.hpp>
#include <sm/assert_macros.hpp>
#include <sm/boost/serialization.hpp>
#include <aslam/cameras/GridCalibrationTargetBase.hpp>

namespace aslam {
namespace cameras {

class GridCalibrationTargetGeneral : public GridCalibrationTargetBase {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  typedef boost::shared_ptr<GridCalibrationTargetGeneral> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetGeneral> ConstPtr;

  /// \brief initialize based on circlegrid geometry
  GridCalibrationTargetGeneral(size_t rows, size_t cols);

  virtual ~GridCalibrationTargetGeneral() {};

  /// \brief extract the calibration target points from an image and write to an observation
  bool computeObservation(const cv::Mat &image, Eigen::MatrixXd &outImagePoints,
                          std::vector<bool> &outCornerObserved) const;

 private:
  /// \brief initialize the object
  void initialize();

  /// \brief initialize the grid with the points
  void createGridPoints();


  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
 public:
  enum {CLASS_SERIALIZATION_VERSION = 1};

  //serialization ctor
  GridCalibrationTargetGeneral() {};

 protected:
  friend class boost::serialization::access;

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    boost::serialization::void_cast_register<GridCalibrationTargetGeneral, GridCalibrationTargetBase>(
            static_cast<GridCalibrationTargetGeneral *>(NULL),
            static_cast<GridCalibrationTargetBase *>(NULL));
    ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
  }
  template<class Archive>
  void load(Archive & ar, const unsigned int /* version */) {
    boost::serialization::void_cast_register<GridCalibrationTargetGeneral, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetGeneral *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    initialize();
  }

};
}  // namespace cameras
}  // namespace aslam


SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetGeneral);
BOOST_CLASS_EXPORT_KEY(aslam::cameras::GridCalibrationTargetGeneral);
#endif /* ASLAM_GRID_CALIBRATION_TARGET_GENERAL_HPP */
