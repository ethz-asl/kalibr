#ifndef ASLAM_FRAME_BASE_HPP
#define ASLAM_FRAME_BASE_HPP

#include <aslam/Time.hpp>
#include <Eigen/Core>
#include <aslam/KeypointBase.hpp>
#include <opencv2/core/core.hpp>
#include <aslam/BackProjection.hpp>
#include <sm/kinematics/UncertainHomogeneousPoint.hpp>
#include <boost/serialization/split_member.hpp>
#include <aslam/Image.hpp>
#include <boost/shared_ptr.hpp>

namespace aslam {

struct KeypointIdentifier;
namespace cameras {
class CameraGeometryBase;
}  // namespace cameras

class FrameBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef boost::shared_ptr<FrameBase> Ptr;

  FrameBase();
  virtual ~FrameBase();

  // The camera geometry base class.
  virtual boost::shared_ptr<cameras::CameraGeometryBase> geometryBase() = 0;
  virtual boost::shared_ptr<const cameras::CameraGeometryBase> geometryBase() const = 0;

  /// \brief this will throw if the dynamic cast to the underlying type fails.
  virtual void setGeometryBase(
      const boost::shared_ptr<cameras::CameraGeometryBase> & geometry) = 0;

  /// \brief this will clear the camera geometry and make this frame object not that useful.
  ///        do you really want to do this?
  virtual void clearGeometry() = 0;

  /// \brief clear the keypoints in the frame
  virtual void clearKeypoints() = 0;

  // Just like the camera class, these are variable size functions.
  // The fixed size functions are available in the derived class
  virtual const KeypointBase & keypointBase(size_t i) const = 0;
  virtual KeypointBase & keypointBase(size_t i) = 0;
  virtual size_t numKeypoints() const = 0;

  // The keypoint time. This will be something like: frame.time() + geometry.temporalOffset( keypoint(i) );
  virtual Time keypointTime(size_t i) const = 0;

  virtual const FrameId & id() const = 0;
  virtual void setId(const FrameId & id) = 0;

  // The base time for this frame. i.e. the time at which integration started.
  virtual const Time & time() const = 0;
  virtual void setTime(const Time & time) = 0;

  // Should we also implement drawing keypoints on the image using the camera model?

  virtual const cv::Mat& image(int octave = 0) const = 0;

  // Set the image
  virtual void setImage(const cv::Mat & image, int octave = 0) = 0;

  //get the image octaves. The underlying aslam::Image
  virtual boost::shared_ptr<const aslam::Image> getOctaves() const = 0;

  //get the image octaves. The underlying aslam::Image non const
  virtual boost::shared_ptr<aslam::Image> getOctavesMutable() = 0;

  ///////////////////////////////////////////////////
  // Reprojection error
  ///////////////////////////////////////////////////

  /// \brief compute the reprojection error for this keypoint from a Euclidean point
  ///        returns true if the reprojection was successful
  virtual bool computeReprojectionError3(
      size_t i, const Eigen::Vector3d & p,
      double & outReprojectionError) const = 0;

  /// \brief compute the reprojection error for this keypoint from a Homogeneous point
  ///        returns true if the reprojection was successful
  virtual bool computeReprojectionError4(
      size_t i, const Eigen::Vector4d & p,
      double & outReprojectionError) const = 0;

  /// \brief compute the reprojection error for this keypoint from an uncertain Homogeneous point
  ///        returns true if the reprojection was successful
  virtual bool computeReprojectionErrorUhp(
      size_t i, const sm::kinematics::UncertainHomogeneousPoint & p,
      double & outReprojectionError) const = 0;

  /// \brief compute the reprojection for a 4x1 homogeneous point
  ///        returns true if the projection was successful
  virtual bool computeProjection4(const Eigen::Vector4d & ph,
                                  Eigen::VectorXd & outReprojection) const = 0;
  /// \brief compute the reprojection for a 3x1 point
  ///        returns true if the projection was successful
  virtual bool computeProjection3(const Eigen::Vector3d & ph,
                                  Eigen::VectorXd & outReprojection) const = 0;
  /// \brief compute the reprojection for an uncertain homogeneous point
  ///        returns true if the projection was successful
  virtual bool computeProjectionUhp(
      const sm::kinematics::UncertainHomogeneousPoint & ph,
      Eigen::VectorXd & outReprojection) const = 0;

  ///////////////////////////////////////////////////
  // A Generic interface for camera systems
  ///////////////////////////////////////////////////

  /// \brief compute the reprojection error for this keypoint from a Euclidean point
  ///        returns true if the reprojection was successful
  virtual bool computeReprojectionError3(
      const KeypointIdentifier & kid, const Eigen::Vector3d & p,
      double & outReprojectionError) const = 0;
  /// \brief compute the reprojection error for this keypoint from a Homogeneous point
  ///        returns true if the reprojection was successful
  virtual bool computeReprojectionError4(
      const KeypointIdentifier & kid, const Eigen::Vector4d & ph,
      double & outReprojectionError) const = 0;
  /// \brief compute the reprojection error for this keypoint from an uncertain Homogeneous point
  ///        returns true if the reprojection was successful
  virtual bool computeReprojectionErrorUhp(
      const KeypointIdentifier & kid,
      const sm::kinematics::UncertainHomogeneousPoint & p,
      double & outReprojectionError) const = 0;

  /// \brief The keypoint time. This will be something like: frame.time() + geometry.temporalOffset( keypoint(i) );
  virtual Time keypointTime(const KeypointIdentifier & kid) const = 0;

  /// \brief get the back projection for keypoint i
  virtual void getBackProjection(size_t i,
                                 BackProjection & outBackProjection) const = 0;

  /// \brief get the back projection for keypoint kid
  virtual void getBackProjection(const KeypointIdentifier & kid,
                                 BackProjection & outBackProjection) const = 0;

  /// \brief get the back projection for keypoint i
  virtual void getUncertainBackProjection(
      size_t i, UncertainBackProjection & outBackProjection) const = 0;

  /// \brief get the back projection for keypoint kid
  virtual void getUncertainBackProjection(
      const KeypointIdentifier & kid,
      UncertainBackProjection & outBackProjection) const = 0;

  /// \brief compute and store all back projections.
  virtual void computeAllBackProjections(bool doBackProjectionUncertainty) = 0;

  // \todo Add getLandmark() functions that include uncertainty
  //         virtual bool getLandmark(size_t k, HomogeneousPoint & outLandmark) const = 0;
  //         virtual bool getLandmark(const KeypointIdentifier & k, HomogeneousPoint & outLandmark) const = 0;
  //         virtual bool getLandmark(const KeypointIdentifier & k, UncertainHomogeneousPoint & outLandmark) const = 0;
  //         virtual bool getLandmark(size_t k, UncertainHomogeneousPoint & outLandmark) const = 0;

  /// \brief get the landmark for keypoint k. Return true on success
  virtual bool getLandmark(size_t k, Eigen::Vector4d & outLandmark) const = 0;

  /// \brief get the landmark for keypoint k. Return true on success
  virtual bool getLandmark(const KeypointIdentifier kid,
                           Eigen::Vector4d & outLandmark) const = 0;

  virtual bool isBinaryEqual(const FrameBase & rhs) const = 0;

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
  virtual void addBaseKeypoints(const KeypointBase * kp, size_t n) = 0;

  /** 
   * Add a keypoint to the array. The type will be checked with dynamic cast
   * 
   * @param kp 
   */
  virtual void addBaseKeypoint(const KeypointBase * kp) = 0;

  virtual KeypointBase & addBaseKeypoint() = 0;

  /// \brief get the number of keypoints that have a landmark ID set
  virtual size_t numKeypointsWithALandmarkId() const;

  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
  enum {
    CLASS_SERIALIZATION_VERSION = 2
  };BOOST_SERIALIZATION_SPLIT_MEMBER();

  template<class Archive>
  void load(Archive & /* ar */, const unsigned int /* version */) {
  }

  template<class Archive>
  void save(Archive & /* ar */, const unsigned int /* version */) const {
  }

 private:

};

}  // namespace aslam

SM_BOOST_CLASS_VERSION (aslam::FrameBase);

#endif /* ASLAM_FRAME_BASE_HPP */
