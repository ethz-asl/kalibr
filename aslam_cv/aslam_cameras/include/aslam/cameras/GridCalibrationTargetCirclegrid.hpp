#ifndef ASLAM_GRID_CALIBRATION_TARGET_CIRCLEGRID_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_CIRCLEGRID_HPP

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

class GridCalibrationTargetCirclegrid : public GridCalibrationTargetBase {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  typedef boost::shared_ptr<GridCalibrationTargetCirclegrid> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetCirclegrid> ConstPtr;

  //target extraction options
  struct CirclegridOptions {
    CirclegridOptions() :
      useAsymmetricCirclegrid(false),
      showExtractionVideo(false) {};

    /// \brief asymmetric circlegrid (-->opencv)
    bool useAsymmetricCirclegrid;

    /// \brief show extracted corners
    bool showExtractionVideo;

    /// \brief Serialization support
    enum {CLASS_SERIALIZATION_VERSION = 1};
    BOOST_SERIALIZATION_SPLIT_MEMBER()
    template<class Archive>
    void save(Archive & ar, const unsigned int /*version*/) const
    {
       ar << BOOST_SERIALIZATION_NVP(useAsymmetricCirclegrid);
       ar << BOOST_SERIALIZATION_NVP(showExtractionVideo);
    }
    template<class Archive>
    void load(Archive & ar, const unsigned int /*version*/)
    {
       ar >> BOOST_SERIALIZATION_NVP(useAsymmetricCirclegrid);
       ar >> BOOST_SERIALIZATION_NVP(showExtractionVideo);
    }
  };

  /// \brief initialize based on circlegrid geometry
  GridCalibrationTargetCirclegrid(size_t rows, size_t cols, double spacingMeters,
                                  const GridCalibrationTargetCirclegrid::CirclegridOptions &options = CirclegridOptions());

  virtual ~GridCalibrationTargetCirclegrid() {};

  /// \brief extract the calibration target points from an image and write to an observation
  bool computeObservation(const cv::Mat &image, Eigen::MatrixXd &outImagePoints,
                          std::vector<bool> &outCornerObserved) const;

 private:
  /// \brief initialize the object
  void initialize();

  /// \brief initialize the grid with the points
  void createGridPoints();

  /// \brief size of a circlegrid "square" [m]
  double _spacing;

  /// \brief grid options
  CirclegridOptions _options;

  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
 public:
  enum {CLASS_SERIALIZATION_VERSION = 1};
  BOOST_SERIALIZATION_SPLIT_MEMBER()

  //serialization ctor
  GridCalibrationTargetCirclegrid() {};

 protected:
  friend class boost::serialization::access;

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    boost::serialization::void_cast_register<GridCalibrationTargetCirclegrid, GridCalibrationTargetBase>(
            static_cast<GridCalibrationTargetCirclegrid *>(NULL),
            static_cast<GridCalibrationTargetBase *>(NULL));
    ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar << BOOST_SERIALIZATION_NVP(_spacing);
    ar << BOOST_SERIALIZATION_NVP(_options);
  }
  template<class Archive>
  void load(Archive & ar, const unsigned int /* version */) {
    boost::serialization::void_cast_register<GridCalibrationTargetCirclegrid, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetCirclegrid *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar >> BOOST_SERIALIZATION_NVP(_spacing);
    ar >> BOOST_SERIALIZATION_NVP(_options);
    initialize();
  }
};

}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetCirclegrid);
SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetCirclegrid::CirclegridOptions);
BOOST_CLASS_EXPORT_KEY(aslam::cameras::GridCalibrationTargetCirclegrid);

#endif /* ASLAM_GRID_CALIBRATION_TARGET_CIRCLEGRID_HPP */
