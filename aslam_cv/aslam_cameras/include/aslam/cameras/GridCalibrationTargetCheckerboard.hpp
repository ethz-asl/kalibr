#ifndef ASLAM_GRID_CALIBRATION_TARGET_CHECKERBOARD_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_CHECKERBOARD_HPP

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

class GridCalibrationTargetCheckerboard : public GridCalibrationTargetBase {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  typedef boost::shared_ptr<GridCalibrationTargetCheckerboard> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetCheckerboard> ConstPtr;

  //target extraction options
  struct CheckerboardOptions {
    CheckerboardOptions() :
      useAdaptiveThreshold(true),
      normalizeImage(true),
      performFastCheck(true),
      filterQuads(false),
      doSubpixelRefinement(true),
      showExtractionVideo(false),
      windowWidth(11) {};

    /// \brief opencv options
    bool useAdaptiveThreshold;
    bool normalizeImage;
    bool performFastCheck;
    bool filterQuads;
    bool doSubpixelRefinement;
    unsigned int windowWidth;

    /// \brief show extracted corners
    bool showExtractionVideo;

    /// \brief Serialization support
    enum {CLASS_SERIALIZATION_VERSION = 1};
    BOOST_SERIALIZATION_SPLIT_MEMBER()
    template<class Archive>
    void save(Archive & ar, const unsigned int /*version*/) const
    {
       ar << BOOST_SERIALIZATION_NVP(useAdaptiveThreshold);
       ar << BOOST_SERIALIZATION_NVP(normalizeImage);
       ar << BOOST_SERIALIZATION_NVP(performFastCheck);
       ar << BOOST_SERIALIZATION_NVP(filterQuads);
       ar << BOOST_SERIALIZATION_NVP(doSubpixelRefinement);
       ar << BOOST_SERIALIZATION_NVP(showExtractionVideo);
       ar << BOOST_SERIALIZATION_NVP(windowWidth);
    }
    template<class Archive>
    void load(Archive & ar, const unsigned int /*version*/)
    {
       ar >> BOOST_SERIALIZATION_NVP(useAdaptiveThreshold);
       ar >> BOOST_SERIALIZATION_NVP(normalizeImage);
       ar >> BOOST_SERIALIZATION_NVP(performFastCheck);
       ar >> BOOST_SERIALIZATION_NVP(filterQuads);
       ar >> BOOST_SERIALIZATION_NVP(doSubpixelRefinement);
       ar >> BOOST_SERIALIZATION_NVP(showExtractionVideo);
       ar >> BOOST_SERIALIZATION_NVP(windowWidth);
    }
  };

  /// \brief initialize based on checkerboard geometry
  GridCalibrationTargetCheckerboard(size_t rows, size_t cols, double rowSpacingMeters,
                                    double colSpacingMeters,
                                    const GridCalibrationTargetCheckerboard::CheckerboardOptions &options = CheckerboardOptions());

  virtual ~GridCalibrationTargetCheckerboard() {};

  /// \brief extract the calibration target points from an image and write to an observation
  bool computeObservation(const cv::Mat &image, Eigen::MatrixXd &outImagePoints,
                          std::vector<bool> &outCornerObserved) const;

 private:
  /// \brief initialize the object
  void initialize();

  /// \brief initialize the grid with the points
  void createGridPoints();

  /// \brief size of a checkerboard square in rows direction [m]
  double _rowSpacingMeters;

  /// \brief size of a checkerboard square in cols direction [m]
  double _colSpacingMeters;

  /// \brief checkerboard extraction options
  CheckerboardOptions _options;

  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
 public:
  enum {CLASS_SERIALIZATION_VERSION = 1};
  BOOST_SERIALIZATION_SPLIT_MEMBER()

  //serialization ctor
  GridCalibrationTargetCheckerboard() {};

 protected:
  friend class boost::serialization::access;

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    boost::serialization::void_cast_register<GridCalibrationTargetCheckerboard, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetCheckerboard *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar << BOOST_SERIALIZATION_NVP(_rowSpacingMeters);
    ar << BOOST_SERIALIZATION_NVP(_colSpacingMeters);
    ar << BOOST_SERIALIZATION_NVP(_options);
  }
  template<class Archive>
  void load(Archive & ar, const unsigned int /* version */) {
    boost::serialization::void_cast_register<GridCalibrationTargetCheckerboard, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetCheckerboard *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar >> BOOST_SERIALIZATION_NVP(_rowSpacingMeters);
    ar >> BOOST_SERIALIZATION_NVP(_colSpacingMeters);
    ar >> BOOST_SERIALIZATION_NVP(_options);
    initialize();
  }
};

}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetCheckerboard);
SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetCheckerboard::CheckerboardOptions);
BOOST_CLASS_EXPORT_KEY(aslam::cameras::GridCalibrationTargetCheckerboard);

#endif /* ASLAM_GRID_CALIBRATION_TARGET_CHECKERBOARD_HPP */
