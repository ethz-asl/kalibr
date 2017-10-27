#ifndef ASLAM_GRID_CALIBRATION_TARGET_ASSYMETRIC_APRILGRID_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_ASSYMETRIC_APRILGRID_HPP

#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetBase.hpp>
#include "apriltags/AllTags.h" //make script for including auto when generating library?????
#include "apriltags/TagDetector.h"
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/python.hpp>

#define TAGCODES AprilTags::tagCodes25h4

namespace aslam {
namespace cameras {

class GridCalibrationTargetAssymetricAprilgrid : public GridCalibrationTargetBase {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  typedef boost::shared_ptr<GridCalibrationTargetAssymetricAprilgrid> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetAssymetricAprilgrid> ConstPtr;
  
 struct TargetPoint {
    TargetPoint():
      x(-1),
      y(-1),
      size(0),
      row{},
      col{},
      id(-1) {};
 
    float x,y;
    unsigned int size;
    std::array<int, 2> row, col;
    int id;
  
     /// \brief Serialization support
    enum {CLASS_SERIALIZATION_VERSION = 1};
    BOOST_SERIALIZATION_SPLIT_MEMBER();
    template<class Archive>
    void save(Archive & ar, const unsigned int /*version*/) const
    {
       ar << BOOST_SERIALIZATION_NVP(x);
       ar << BOOST_SERIALIZATION_NVP(y);
       ar << BOOST_SERIALIZATION_NVP(size);
       ar << BOOST_SERIALIZATION_NVP(row);
       ar << BOOST_SERIALIZATION_NVP(col);
       ar << BOOST_SERIALIZATION_NVP(id);
    }
    template<class Archive>
    void load(Archive & ar, const unsigned int /*version*/)
    {
       ar >> BOOST_SERIALIZATION_NVP(x);
       ar >> BOOST_SERIALIZATION_NVP(y);
       ar >> BOOST_SERIALIZATION_NVP(size);
       ar >> BOOST_SERIALIZATION_NVP(row);
       ar >> BOOST_SERIALIZATION_NVP(col);
       ar >> BOOST_SERIALIZATION_NVP(id);
    }


};
 
   
  /// \brief initialize based on checkerboard geometry
 GridCalibrationTargetAssymetricAprilgrid(const std::vector<TargetPoint> &targetPoints,
    const GridCalibrationTargetAprilgrid::AprilgridOptions &options = GridCalibrationTargetAprilgrid::AprilgridOptions());
  
 GridCalibrationTargetAssymetricAprilgrid(const boost::python::list& vector_list,
    const GridCalibrationTargetAprilgrid::AprilgridOptions &options);
  
  virtual ~GridCalibrationTargetAssymetricAprilgrid() {};

  /// \brief extract the calibration target points from an image and write to an observation
   bool computeObservation(const cv::Mat & image,
                          Eigen::MatrixXd & outImagePoints,
                          std::vector<bool> &outCornerObserved) const;
 
 private:
  void initialize();
  void createGridPoints();
  void targetPoints2Grid();
  std::vector<TargetPoint> _targetPoints;
  GridCalibrationTargetAprilgrid::AprilgridOptions _options;

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
  GridCalibrationTargetAssymetricAprilgrid();

 protected:
  friend class boost::serialization::access;

  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    boost::serialization::void_cast_register<GridCalibrationTargetAssymetricAprilgrid, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetAssymetricAprilgrid *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar << BOOST_SERIALIZATION_NVP(_targetPoints);
    ar << BOOST_SERIALIZATION_NVP(_options);

  }
  template<class Archive>
  void load(Archive & ar, const unsigned int /* version */) {
    boost::serialization::void_cast_register<GridCalibrationTargetAssymetricAprilgrid, GridCalibrationTargetBase>(
          static_cast<GridCalibrationTargetAssymetricAprilgrid *>(NULL),
          static_cast<GridCalibrationTargetBase *>(NULL));
    ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar >> BOOST_SERIALIZATION_NVP(_targetPoints);
    ar >> BOOST_SERIALIZATION_NVP(_options);

    initialize();
  }
};


}  // namespace cameras
}  // namespace aslam


SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetAssymetricAprilgrid);
SM_BOOST_CLASS_VERSION(aslam::cameras::GridCalibrationTargetAssymetricAprilgrid::TargetPoint);
BOOST_CLASS_EXPORT_KEY(aslam::cameras::GridCalibrationTargetAssymetricAprilgrid)

#endif /* ASLAM_GRID_CALIBRATION_TARGET_ASSYMETRIC_APRILGRID_HPP */

