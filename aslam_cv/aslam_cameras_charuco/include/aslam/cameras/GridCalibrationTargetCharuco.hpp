#ifndef ASLAM_GRID_CALIBRATION_TARGET_CHARUCO_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_CHARUCO_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <sm/assert_macros.hpp>
#include <aslam/cameras/GridCalibrationTargetBase.hpp>
#include <boost/serialization/export.hpp>

namespace aslam {
namespace cameras {

class GridCalibrationTargetCharuco : public GridCalibrationTargetBase {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  typedef boost::shared_ptr<GridCalibrationTargetCharuco> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetCharuco> ConstPtr;

  //target extraction options
  struct CharucoOptions {
    CharucoOptions() :
      doSubpixRefinement(true),
      maxSubpixDisplacement2(1.5),
      showExtractionVideo(false),
      minTagsForValidObs(2),
      minBorderDistance(3.0),
      blackTagBorder(1) {};

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
    unsigned int minBorderDistance;

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
  GridCalibrationTargetCharuco(size_t tagRows, size_t tagCols, double tagSize,
                               double tagSpacing, const std::string& dictName,
                               size_t markerSize, size_t nMarkers,
                               const CharucoOptions &options = CharucoOptions());

  virtual ~GridCalibrationTargetCharuco() {};

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

  /// \brief Name of the Charuco dictionary (one of PREDEFINED_DICTIONARY_NAME without the DICT_ prefix).
  std::string _dictName;

  /// \brief Bit size of marker (used only when _dictName is empty).
  size_t _markerSize;

  /// \brief Number of markers in dictionary (used only when _dictName is empty).
  size_t _nMarkers;

  /// \brief target extraction options
  CharucoOptions _options;

  cv::Ptr<cv::aruco::Dictionary> _dict;
  cv::Ptr<cv::aruco::CharucoBoard> _board;
  cv::Ptr<cv::aruco::DetectorParameters> _params;

  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
 public:
  enum {CLASS_SERIALIZATION_VERSION = 1};
  BOOST_SERIALIZATION_SPLIT_MEMBER()

  //serialization ctor
  GridCalibrationTargetCharuco();

 protected:
  friend class boost::serialization::access;

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    boost::serialization::void_cast_register<GridCalibrationTargetCharuco, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetCharuco *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar << BOOST_SERIALIZATION_NVP(_tagSize);
    ar << BOOST_SERIALIZATION_NVP(_tagSpacing);
    ar << BOOST_SERIALIZATION_NVP(_options);
    ar << BOOST_SERIALIZATION_NVP(_dictName);
    ar << BOOST_SERIALIZATION_NVP(_markerSize);
    ar << BOOST_SERIALIZATION_NVP(_nMarkers);
  }
  template<class Archive>
  void load(Archive & ar, const unsigned int /* version */) {
    boost::serialization::void_cast_register<GridCalibrationTargetCharuco, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetCharuco *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar >> BOOST_SERIALIZATION_NVP(_tagSize);
    ar >> BOOST_SERIALIZATION_NVP(_tagSpacing);
    ar >> BOOST_SERIALIZATION_NVP(_options);
    ar >> BOOST_SERIALIZATION_NVP(_dictName);
    ar >> BOOST_SERIALIZATION_NVP(_markerSize);
    ar >> BOOST_SERIALIZATION_NVP(_nMarkers);
    initialize();
  }
};


}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetCharuco);
SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetCharuco::CharucoOptions);
BOOST_CLASS_EXPORT_KEY(aslam::cameras::GridCalibrationTargetCharuco)

#endif /* ASLAM_GRID_CALIBRATION_TARGET_CHARUCO_HPP */
