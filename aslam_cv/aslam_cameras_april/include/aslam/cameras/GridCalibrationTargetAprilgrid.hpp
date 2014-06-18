#ifndef ASLAM_GRID_CALIBRATION_TARGET_APRILGRID_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_APRILGRID_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <sm/assert_macros.hpp>
#include <aslam/cameras/GridCalibrationTargetBase.hpp>
#include <boost/serialization/export.hpp>

// April tags detector and various tag families
#include "apriltags/TagDetector.h"
//#include "apriltags/Tag16h5.h"
//#include "apriltags/Tag25h7.h"
//#include "apriltags/Tag25h9.h"
//#include "apriltags/Tag36h9.h"
#include "apriltags/Tag36h11.h"

namespace aslam {
namespace cameras {

class GridCalibrationTargetAprilgrid : public GridCalibrationTargetBase {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  typedef boost::shared_ptr<GridCalibrationTargetAprilgrid> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetAprilgrid> ConstPtr;

  //target extraction options
  struct AprilgridOptions {
    AprilgridOptions() :
      doSubpixRefinement(true),
      maxSubpixDisplacement2(1.5),
      showExtractionVideo(false),
      minTagsForValidObs(4),
      minBorderDistance(4.0),
      blackTagBorder(2) {};

    //options
    /// \brief subpixel refinement of extracted corners
    bool doSubpixRefinement;

    /// \brief max. displacement squarred in subpixel refinement  [px^2]
    double maxSubpixDisplacement2;

    /// \brief show video during extraction
    bool showExtractionVideo;

    /// \brief min. number of tags for a valid observation
    unsigned int minTagsForValidObs;

    /// \brief min. distance form image border for valid points [px]
    double minBorderDistance;

    /// \brief size of black border around the tag code bits (in pixels)
    unsigned int blackTagBorder;

    /// \brief Serialization support
    enum {CLASS_SERIALIZATION_VERSION = 1};
    BOOST_SERIALIZATION_SPLIT_MEMBER();
    template<class Archive>
    void save(Archive & ar, const unsigned int /*version*/) const
    {
       ar << BOOST_SERIALIZATION_NVP(doSubpixRefinement);
       ar << BOOST_SERIALIZATION_NVP(maxSubpixDisplacement2);
       ar << BOOST_SERIALIZATION_NVP(showExtractionVideo);
       ar << BOOST_SERIALIZATION_NVP(minTagsForValidObs);
       ar << BOOST_SERIALIZATION_NVP(minBorderDistance);
       ar << BOOST_SERIALIZATION_NVP(blackTagBorder);
    }
    template<class Archive>
    void load(Archive & ar, const unsigned int /*version*/)
    {
       ar >> BOOST_SERIALIZATION_NVP(doSubpixRefinement);
       ar >> BOOST_SERIALIZATION_NVP(maxSubpixDisplacement2);
       ar >> BOOST_SERIALIZATION_NVP(showExtractionVideo);
       ar >> BOOST_SERIALIZATION_NVP(minTagsForValidObs);
       ar >> BOOST_SERIALIZATION_NVP(minBorderDistance);
       ar >> BOOST_SERIALIZATION_NVP(blackTagBorder);
    }
  };

  /// \brief initialize based on checkerboard geometry
  GridCalibrationTargetAprilgrid(size_t tagRows, size_t tagCols, double tagSize,
                                 double tagSpacing, const AprilgridOptions &options = AprilgridOptions());

  virtual ~GridCalibrationTargetAprilgrid() {};

  /// \brief extract the calibration target points from an image and write to an observation
  bool computeObservation(const cv::Mat & image,
                          Eigen::MatrixXd & outImagePoints,
                          std::vector<bool> &outCornerObserved) const;

 private:
  /// \brief initialize the object
  void initialize();

  /// \brief initialize the grid with the points
  void createGridPoints();

  /// \brief size of a tag [m]
  double _tagSize;

  /// \brief space between tags (tagSpacing [m] = tagSize * tagSpacing)
  double _tagSpacing;

  /// \brief target extraction options
  AprilgridOptions _options;

  // create a detector instance
  AprilTags::TagCodes _tagCodes;
  boost::shared_ptr<AprilTags::TagDetector> _tagDetector;

  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
 public:
  enum {CLASS_SERIALIZATION_VERSION = 1};
  BOOST_SERIALIZATION_SPLIT_MEMBER()

  //serialization ctor
  GridCalibrationTargetAprilgrid();

 protected:
  friend class boost::serialization::access;

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    boost::serialization::void_cast_register<GridCalibrationTargetAprilgrid, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetAprilgrid *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar << BOOST_SERIALIZATION_NVP(_tagSize);
    ar << BOOST_SERIALIZATION_NVP(_tagSpacing);
    ar << BOOST_SERIALIZATION_NVP(_options);
  }
  template<class Archive>
  void load(Archive & ar, const unsigned int /* version */) {
    boost::serialization::void_cast_register<GridCalibrationTargetAprilgrid, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetAprilgrid *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar >> BOOST_SERIALIZATION_NVP(_tagSize);
    ar >> BOOST_SERIALIZATION_NVP(_tagSpacing);
    ar >> BOOST_SERIALIZATION_NVP(_options);
    initialize();
  }
};


}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetAprilgrid);
SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetAprilgrid::AprilgridOptions);
BOOST_CLASS_EXPORT_KEY(aslam::cameras::GridCalibrationTargetAprilgrid)

#endif /* ASLAM_GRID_CALIBRATION_TARGET_APRILGRID_HPP */

