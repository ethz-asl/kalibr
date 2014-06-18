/**
 * @file   Frame.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>
 * @date   Sat Nov 24 20:25:39 2012
 * 
 * @brief  A class representing a single image with keypoints
 * 
 * 
 */

#ifndef ASLAM_FRAME_HPP
#define ASLAM_FRAME_HPP

#include <aslam/Time.hpp>
#include "Keypoint.hpp"
#include "FrameBase.hpp"
#include <Eigen/StdVector>

#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>
#include <sm/eigen/serialization.hpp>
#include <sm/opencv/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/boost/serialization.hpp>
#include <aslam/utilities.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>

namespace aslam {

template<typename CAMERA_GEOMETRY_T>
class Frame : public FrameBase {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);SM_DEFINE_EXCEPTION(IndexOutOfBoundsException, Exception);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief the type of camera geometry.
  typedef CAMERA_GEOMETRY_T camera_geometry_t;

  enum {
    /// \brief the dimensionality of the keypoint
    KeypointDimension = camera_geometry_t::KeypointDimension
  };

  /// \brief the keypoint type
  typedef Keypoint<KeypointDimension> keypoint_t;

  /// \brief the measurement type
  typedef typename keypoint_t::measurement_t measurement_t;

  /// \brief a type for a list of keypoints
  typedef std::vector<keypoint_t, Eigen::aligned_allocator<keypoint_t> > keypoint_list_t;

  /// \brief a default constructor
  Frame();

  /// \brief a simple desctuctor
  virtual ~Frame();

  ///////////////////////////////////////////////////
  // Camera Geometry
  ///////////////////////////////////////////////////

  /// \brief the geometry object for the keypoint
  const camera_geometry_t & geometry() const;

  /// \brief the underlying shared pointer for the camera geometry
  boost::shared_ptr<camera_geometry_t> geometryPtr() const;

  /// \brief set the camera geometry
  void setGeometry(
      const boost::shared_ptr<camera_geometry_t>& cameraGeometryPtr);

  /// \brief get the base camera geometry
  virtual boost::shared_ptr<cameras::CameraGeometryBase> geometryBase();

  /// \brief get the base camera geometry
  virtual boost::shared_ptr<const cameras::CameraGeometryBase> geometryBase() const;

  /// \brief this will throw if the dynamic cast to the underlying type fails.
  virtual void setGeometryBase(
      const boost::shared_ptr<cameras::CameraGeometryBase> & geometry);

  /// \brief this will clear the camera geometry and make this frame object not that useful.
  ///        do you really want to do this?
  virtual void clearGeometry();

  ///////////////////////////////////////////////////
  // Keypoints
  ///////////////////////////////////////////////////

  /// \brief get the number of keypoints
  virtual size_t numKeypoints() const;

  /// \brief get the number of keypoints
  virtual size_t size() const {
    return numKeypoints();
  }

  /// \brief clear the keypoints in the frame
  virtual void clearKeypoints();

  /// \brief Get keypoint i
  const keypoint_t & keypoint(size_t i) const;

  /// \brief get keypoint i
  keypoint_t & keypoint(size_t i);

  /// \brief get the keypoint base object
  virtual const KeypointBase & keypointBase(size_t i) const;

  /// \brief get the keypoint base object
  virtual KeypointBase & keypointBase(size_t i);

  /// \brief get keypoint i and check that i is in bounds
  keypoint_t & keypointChecked(size_t i);

  /// \brief get keypoint i and check that i is in bounds
  const keypoint_t & keypointChecked(size_t i) const;

  /// \brief get keypoint i and do not check that i is in bounds (fast version)
  inline keypoint_t & keypointUnchecked(size_t i) {
    return _keypoints[i];
  }

  /// \brief get keypoint i and do not check that i is in bounds (fast version)
  inline const keypoint_t & keypointUnchecked(size_t i) const {
    return _keypoints[i];
  }

  /// \brief get keypoint i
  const keypoint_t & operator[](size_t i) const {
    return keypoint(i);
  }

  /// \brief get keypoint i
  keypoint_t & operator[](size_t i) {
    return keypoint(i);
  }

  /// \brief Keypoints can only be added, never removed. This is because data associations between
  ///        frames are stored as indices.
  keypoint_t & addKeypoint();

  /// \brief add a keypoint
  void addKeypoint(const keypoint_t & keypoint);

  virtual KeypointBase & addBaseKeypoint();

  /// \brief add a list of keypoints
  void addKeypoints(const keypoint_list_t & keypoints);

  /// \brief reserves space for keypoints
  void reserveKeypoints(size_t size);

  ///////////////////////////////////////////////////
  // Reprojection error
  ///////////////////////////////////////////////////

  /// \brief compute the reprojection error for this keypoint from a Euclidean point
  ///        returns true if the reprojection was successful
  virtual bool computeReprojectionError3(size_t i, const Eigen::Vector3d & p,
                                         double & outReprojectionError) const;

  /// \brief compute the reprojection error for this keypoint from a Homogeneous point
  ///        returns true if the reprojection was successful
  virtual bool computeReprojectionError4(size_t i, const Eigen::Vector4d & p,
                                         double & outReprojectionError) const;

  /// \brief compute the reprojection error for this keypoint from an uncertain Homogeneous point
  ///        returns true if the reprojection was successful
  virtual bool computeReprojectionErrorUhp(
      size_t i, const sm::kinematics::UncertainHomogeneousPoint & p,
      double & outReprojectionError) const;

  ///////////////////////////////////////////////////
  // A Generic interface for camera systems
  ///////////////////////////////////////////////////

  /// \brief compute the reprojection error for this keypoint from a Euclidean point
  ///        returns true if the reprojection was successful
  virtual bool computeReprojectionError3(const KeypointIdentifier & kid,
                                         const Eigen::Vector3d & p,
                                         double & outReprojectionError) const;
  /// \brief compute the reprojection error for this keypoint from a Homogeneous point
  ///        returns true if the reprojection was successful
  virtual bool computeReprojectionError4(const KeypointIdentifier & kid,
                                         const Eigen::Vector4d & ph,
                                         double & outReprojectionError) const;
  /// \brief compute the reprojection error for this keypoint from an uncertain Homogeneous point
  ///        returns true if the reprojection was successful
  virtual bool computeReprojectionErrorUhp(
      const KeypointIdentifier & kid,
      const sm::kinematics::UncertainHomogeneousPoint & p,
      double & outReprojectionError) const;

  /// \brief compute the projection for a 4x1 homogeneous point
  ///        returns true if the projection was successful
  virtual bool computeProjection4(const Eigen::Vector4d & ph,
                                  Eigen::VectorXd & outReprojection) const;
  /// \brief compute the projection for a 3x1 point
  ///        returns true if the projection was successful
  virtual bool computeProjection3(const Eigen::Vector3d & ph,
                                  Eigen::VectorXd & outReprojection) const;
  /// \brief compute the projection for an uncertain homogeneous point
  ///        returns true if the projection was successful
  virtual bool computeProjectionUhp(
      const sm::kinematics::UncertainHomogeneousPoint & ph,
      Eigen::VectorXd & outReprojection) const;

  /// \brief get the keypoint associated with this identifier.
  void getKeypoint(const KeypointIdentifier & kid,
                   keypoint_t & outKeypoint) const;

  /// \brief get a const ref to the list of keypoints.
  const keypoint_list_t& getKeypoints() const;

  /// \brief The keypoint time. This will be something like: frame.time() + geometry.temporalOffset( keypoint(i) );
  virtual Time keypointTime(const KeypointIdentifier & kid) const;

  /// \brief get the back projection for keypoint i
  virtual void getBackProjection(size_t i,
                                 BackProjection & outBackProjection) const;

  /// \brief get the back projection for keypoint kid
  virtual void getBackProjection(const KeypointIdentifier & kid,
                                 BackProjection & outBackProjection) const;

  /// \brief get the back projection for keypoint i
  virtual void getUncertainBackProjection(
      size_t i, UncertainBackProjection & outBackProjection) const;

  /// \brief get the back projection for keypoint kid
  virtual void getUncertainBackProjection(
      const KeypointIdentifier & kid,
      UncertainBackProjection & outBackProjection) const;

  /// \brief compute and store all back projections.
  virtual void computeAllBackProjections(bool doBackProjectionUncertainty);

  // \todo Add getLandmark() functions that include uncertainty
  //         virtual bool getLandmark(size_t k, HomogeneousPoint & outLandmark) const = 0;
  //         virtual bool getLandmark(const KeypointIdentifier & k, HomogeneousPoint & outLandmark) const = 0;
  //         virtual bool getLandmark(const KeypointIdentifier & k, UncertainHomogeneousPoint & outLandmark) const = 0;
  //         virtual bool getLandmark(size_t k, UncertainHomogeneousPoint & outLandmark) const = 0;

  /// \brief get the landmark for keypoint k. Return true on success
  virtual bool getLandmark(size_t k, Eigen::Vector4d & outLandmark) const;

  /// \brief get the landmark for keypoint k. Return true on success
  virtual bool getLandmark(const KeypointIdentifier kid,
                           Eigen::Vector4d & outLandmark) const;

  ///////////////////////////////////////////////////
  // The frame ID
  ///////////////////////////////////////////////////

  /// \brief get the frame id
  virtual const FrameId & id() const {
    return _id;
  }

  /// \brief set the frame id
  virtual void setId(const FrameId & id) {
    _id = id;
  }

  ///////////////////////////////////////////////////
  // Timestamp support
  ///////////////////////////////////////////////////

  /// \brief the base time for this frame. i.e. the time at which integration started.
  virtual const Time & time() const {
    return _stamp;
  }

  /// \brief set the base time for this frame
  virtual void setTime(const Time & time) {
    _stamp = time;
  }

  /// \brief The keypoint time. This will be something like: frame.time() + geometry.temporalOffset( keypoint(i) );
  virtual Time keypointTime(size_t i) const;

  ///////////////////////////////////////////////////
  // The underlying image
  ///////////////////////////////////////////////////

  /// \brief get the image associated with this frame
  virtual const cv::Mat& image(int octave = 0) const {
    return _image->getOctave(octave);
  }

  /// \brief Set the image
  virtual void setImage(const cv::Mat & image, int octave = 0) {
    _image->getOctaveMutable(octave) = image;
  }

  virtual boost::shared_ptr<const aslam::Image> getOctaves() const {
    return _image;
  }

  virtual boost::shared_ptr<aslam::Image> getOctavesMutable() {
    return _image;
  }

  /** 
   * \brief add an array of keypoints to the frame.
   *
   * WARNING: This will check the type of the first keypoints in the list
   *          If the check succeeds, it will static cast all of the other
   *          keypoints to the same type. This is bad void * style programming.
   *          but I currently can't think of another way. Be careful. When
   *          in doubt, use the addBaseKeypoint() function.
   * 
   * @param kp An array of keypoints of size N. They are assumed to be all of the same underlying keypoint type.
   * @param n The number of keypoints in the array.
   */
  virtual void addBaseKeypoints(const KeypointBase * kp, size_t n);

  /** 
   * Add a keypoint to the array. The type will be checked with dynamic cast
   * 
   * @param kp 
   */
  virtual void addBaseKeypoint(const KeypointBase * kp);

  ///////////////////////////////////////////////////
  // Serialization and unit test support
  ///////////////////////////////////////////////////
  enum {
    CLASS_SERIALIZATION_VERSION = 2
  };BOOST_SERIALIZATION_SPLIT_MEMBER();

  template<class Archive>
  void load(Archive & ar, const unsigned int version);

  template<class Archive>
  void save(Archive & ar, const unsigned int version) const;

  /// \brief support for unit testing
  virtual bool isBinaryEqual(const Frame<CAMERA_GEOMETRY_T> & rhs) const;

  /// \brief Support for unit testing.
  virtual void setRandom();

  virtual bool isBinaryEqual(const FrameBase & rhs) const;

 private:
  /// \brief a aslam image representing the raw data collected at the frame
  boost::shared_ptr<aslam::Image> _image;

  /// \brief a time stamp
  Time _stamp;

  /// \brief the frame id.
  FrameId _id;

  /// \brief The underlying camera geometry
  boost::shared_ptr<camera_geometry_t> _geometry;

  /// \brief I assume that keypoints may be added but never removed.
  std::vector<keypoint_t, Eigen::aligned_allocator<keypoint_t> > _keypoints;

};

}  // namespace aslam

#include "implementation/Frame.hpp"

// this is a fancy templated version of 
// BOOST_CLASS_VERSION() for serialization versioning
SM_BOOST_CLASS_VERSION_T1 (aslam::Frame);

#endif /* ASLAM_FRAME_HPP */
