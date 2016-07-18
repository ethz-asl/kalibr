#ifndef _NCAMERASYSTEMDESIGNVARIABLECONTAINER_H_
#define _NCAMERASYSTEMDESIGNVARIABLECONTAINER_H_

#include <aslam/NCameraSystem.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/ReprojectionError.hpp>
#include <aslam/backend/MappedRotationQuaternion.hpp>
#include <aslam/backend/MappedEuclideanPoint.hpp>
#include <aslam/MultiFrame.hpp>
#include <boost/shared_ptr.hpp>

namespace aslam {

/// \remarks This class is neither in use nor compiled.
class NCameraSystemDesignVariableContainer {
 public:

  typedef boost::shared_ptr<NCameraSystemDesignVariableContainer> Ptr;

  NCameraSystemDesignVariableContainer(NCameraSystem::Ptr cameraSystem,
                                       bool estimateExtrinsicsRotation,
                                       bool estimateExtrinsicsTranslation,
                                       bool estimateProjection,
                                       bool estimateDistortion,
                                       bool estimateShutter);

  virtual ~NCameraSystemDesignVariableContainer();

  /// \brief Get all design variables associated with this camera system
  void getDesignVariables(backend::DesignVariable::set_t & dvs) const;

  /// \brief Get the associated camera system
  NCameraSystem::Ptr getCameraSystem() const;

  /// \brief set all intrinsics active
  void setIntrinsicsActive(bool active);

  /// \brief set the intrinsics of a specific camera active
  void setCameraIntrinsicsActive(size_t cameraIndex, bool active);

  /// \brief set all extrinsics active
  void setExtrinsicsActive(bool active);

  /// \brief set the intrinsics of a specific camera active
  void setCameraExtrinsicsActive(size_t cameraIndex, bool active);

  /// \brief set all extrinsics active
  void setExtrinsicsRotationActive(bool active);

  /// \brief set all extrinsics active
  void setExtrinsicsTranslationActive(bool active);

  /// \brief set the rotation extrinsics of a specific camera active
  void setCameraRotationExtrinsicsActive(size_t cameraIndex, bool active);

  /// \brief set the translation extrinsics of a specific camera active
  void setCameraTranslationExtrinsicsActive(size_t cameraIndex, bool active);

  /// \brief are the rotation extrinsics of a specific camera active?
  bool isCameraRotationExtrinsicsActive(size_t cameraIndex) const;

  /// \brief are the translation extrinsics of a specific camera active?
  bool isCameraTranslationExtrinsicsActive(size_t cameraIndex) const;

  /// \brief are the intrinsics of a specific camera active?
  bool isCameraIntrinsicsActive(size_t cameraIndex) const;

  /// \brief Create a reprojection error term
  boost::shared_ptr<ReprojectionError>
  createReprojectionError(MultiFrame & mf, const KeypointIdentifier & kid,
                          backend::HomogeneousExpression & p_v);

  backend::TransformationExpression getT_c_v(size_t cameraIndex);

  boost::shared_ptr<CameraGeometryDesignVariableContainer> getCameraGeometryDesignVariableContainer(
      size_t cameraIndex);

  /// \brief save the current values for this camera system
  void saveCurrentValues();

  /// \brief restore the values of this camera system
  void restoreSavedValues();

 private:
  NCameraSystem::Ptr _cameraSystem;

  /// \brief Extrinsics rotation design variables
  std::vector<boost::shared_ptr<backend::MappedRotationQuaternion> > _q_v_c;
  /// \brief Extrinsics translation design variables
  std::vector<boost::shared_ptr<backend::MappedEuclideanPoint> > _p_v_c;
  /// \brief Cached transformation expressions for the extrinsics
  std::vector<backend::TransformationExpression> _T_c_v;

  /// \brief Camera geometry design variables
  std::vector<boost::shared_ptr<CameraGeometryDesignVariableContainer> > _cameraDvs;

  /// \brief Have we ever saved the calibration?
  bool _isSaved;
  std::vector<Eigen::MatrixXd> _savedQuaternions;
  std::vector<Eigen::MatrixXd> _savedEuclidean;
  std::vector<Eigen::MatrixXd> _savedIntrinsics;

};

}  // namespace aslam

#endif /* _NCAMERASYSTEMDESIGNVARIABLECONTAINER_H_ */
