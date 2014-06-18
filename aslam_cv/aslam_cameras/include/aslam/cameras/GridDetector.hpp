#ifndef ASLAM_GRID_DETECTOR_HPP
#define ASLAM_GRID_DETECTOR_HPP

#include <vector>
#include <opencv2/core/core.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/export.hpp>
#include <sm/boost/serialization.hpp>
#include <sm/assert_macros.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/GridCalibrationTargetBase.hpp>

namespace aslam {
namespace cameras {

class GridDetector {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  //target extraction options
  struct GridDetectorOptions {
    GridDetectorOptions() :
      plotCornerReprojection(false),
      imageStepping(false),
      filterCornerOutliers(false),
      filterCornerSigmaThreshold(2.0),
      filterCornerMinReprojError(0.2) {};

    //options
    /// \brief plot the reprojection of the extraced corners during extraction
    bool plotCornerReprojection;

    /// \brief pause after each image for inspection
    bool imageStepping;

    /// \brief filter corner outliers that have a reprojection error above a certain threshold
    bool filterCornerOutliers;

    /// \brief a corner is flagged as inactive, if the its reprojection error is greater than
    ///        mean + filterCornerSigmaThreshold * std  (statistics over all corners on board)
    float filterCornerSigmaThreshold;

    /// \brief filter corner outliers: filtering is only active above this reprojection threshold for a corner
    float filterCornerMinReprojError;

    /// \brief Serialization
    enum {CLASS_SERIALIZATION_VERSION = 1};
    BOOST_SERIALIZATION_SPLIT_MEMBER()

    /// \brief Serialization support
    template<class Archive>
    void save(Archive & ar, const unsigned int /*version*/) const
    {
       ar << BOOST_SERIALIZATION_NVP(plotCornerReprojection);
       ar << BOOST_SERIALIZATION_NVP(imageStepping);
       ar << BOOST_SERIALIZATION_NVP(filterCornerOutliers);
       ar << BOOST_SERIALIZATION_NVP(filterCornerSigmaThreshold);
       ar << BOOST_SERIALIZATION_NVP(filterCornerMinReprojError);
    }
    template<class Archive>
    void load(Archive & ar, const unsigned int /*version*/)
    {
       ar >> BOOST_SERIALIZATION_NVP(plotCornerReprojection);
       ar >> BOOST_SERIALIZATION_NVP(imageStepping);
       ar >> BOOST_SERIALIZATION_NVP(filterCornerOutliers);
       ar >> BOOST_SERIALIZATION_NVP(filterCornerSigmaThreshold);
       ar >> BOOST_SERIALIZATION_NVP(filterCornerMinReprojError);
    }
  };

  /// \brief initialize based on grid geometry
  GridDetector(boost::shared_ptr<CameraGeometryBase> geometry,
               GridCalibrationTargetBase::Ptr target,
               const GridDetector::GridDetectorOptions &options = GridDetectorOptions());

  virtual ~GridDetector();

  /// \brief initialize the detector
  void initializeDetector();

  /// \brief initialize the geometry for a known camera
  void initCameraGeometry(boost::shared_ptr<CameraGeometryBase> geometry);

  /// \brief initialize the geometry from one grid observation
  /// \return true if successful
  bool initCameraGeometryFromObservation(const cv::Mat &image);

  /// \brief initialize the geometry from a list grid observation
  /// \return true if successful
  bool initCameraGeometryFromObservations(boost::shared_ptr<std::vector<cv::Mat> > images_ptr);

  /// \brief get the underlying geometry
  boost::shared_ptr<CameraGeometryBase> geometry() const {
    return _geometry;
  };

  /// \brief get the underlying target
  GridCalibrationTargetBase::Ptr target() const {
    return _target;
  };

  /// \brief Find the target in the image. Return true on success.
  ///
  ///        If the intrinsics are not initialized (geometry pointer null),
  ///        they will be initialized.
  ///        This method will also estimate and fill in the transformation of the
  ///        camera with respect to the grid.
  bool findTarget(const cv::Mat &image, const aslam::Time &stamp,
                  GridCalibrationTargetObservation & outObservation) const;

  bool findTarget(const cv::Mat &image, GridCalibrationTargetObservation &outObservation) const;

  /// \brief Find the target but don't estimate the transformation.
  bool findTargetNoTransformation(const cv::Mat &image, const aslam::Time &stamp,
      GridCalibrationTargetObservation & outObservation) const;

  /// \brief Find the target but don't estimate the transformation.
  bool findTargetNoTransformation(const cv::Mat &image,
                                  GridCalibrationTargetObservation &outObservation) const;

  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
  enum {CLASS_SERIALIZATION_VERSION = 1};
  BOOST_SERIALIZATION_SPLIT_MEMBER()

  /// \brief serialization contstructor (don't use this)
  GridDetector();

 protected:
  friend class boost::serialization::access;

  /// \brief Serialization support
  template<class Archive>
  void save(Archive & ar, const unsigned int /*version*/) const
  {
     ar << BOOST_SERIALIZATION_NVP(_geometry);
     ar << BOOST_SERIALIZATION_NVP(_target);
     ar << BOOST_SERIALIZATION_NVP(_options);
  }

  template<class Archive>
  void load(Archive & ar, const unsigned int /*version*/)
  {
     ar >> BOOST_SERIALIZATION_NVP(_geometry);
     ar >> BOOST_SERIALIZATION_NVP(_target);
     ar >> BOOST_SERIALIZATION_NVP(_options);
     initializeDetector();
  }

 private:
  /// the camera geometry
  boost::shared_ptr<CameraGeometryBase> _geometry;

  /// \brief the calibration target
  GridCalibrationTargetBase::Ptr _target;

  /// \brief detector options
  GridDetectorOptions _options;
};

}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION(aslam::cameras::GridDetector);
SM_BOOST_CLASS_VERSION(aslam::cameras::GridDetector::GridDetectorOptions);

#endif /* ASLAM_GRID_DETECTOR_HPP */
