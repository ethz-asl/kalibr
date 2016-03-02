#include <aslam/NCameraSystemDesignVariableContainer.hpp>
#include <aslam/backend/MapTransformation.hpp>
#include <boost/foreach.hpp>

/// \remarks This file is neither in use nor compiled.
namespace aslam {

NCameraSystemDesignVariableContainer::NCameraSystemDesignVariableContainer(
    NCameraSystem::Ptr cameraSystem, bool estimateExtrinsicsRotation,
    bool estimateExtrinsicsTranslation, bool estimateProjection,
    bool estimateDistortion, bool estimateShutter)
    : _cameraSystem(cameraSystem) {
  SM_ASSERT_TRUE(std::runtime_error, _cameraSystem.get() != NULL,
                 "Cannot initialize with a null camera system");
  _T_c_v.resize(cameraSystem->numCameras());
  // Create mapped extrinsic design variables.
  _q_v_c.resize(cameraSystem->numCameras());
  _p_v_c.resize(cameraSystem->numCameras());
  _savedQuaternions.resize(cameraSystem->numCameras());
  _savedEuclidean.resize(cameraSystem->numCameras());
  _savedIntrinsics.resize(cameraSystem->numCameras());
  for (size_t i = 0; i < cameraSystem->numCameras(); ++i) {

    boost::shared_ptr<sm::kinematics::Transformation> T_v_c = cameraSystem
        ->ptrT_v_c(i);
    _T_c_v[i] = aslam::backend::transformationToExpression(*T_v_c, _q_v_c[i],
                                                           _p_v_c[i]).inverse();
    _q_v_c[i]->setActive(estimateExtrinsicsRotation);
    _p_v_c[i]->setActive(estimateExtrinsicsTranslation);
  }

  // Great! Now for the intrinsics.
  _cameraDvs.resize(cameraSystem->numCameras());
  // CameraGeometryDesignVariableContainer( const boost::shared_ptr<cameras::CameraGeometryBase> & camera,
  //                                        bool estimateProjection,
  //                                        bool estimateDistortion,
  //                                        bool estimateShutter );
  for (size_t i = 0; i < cameraSystem->numCameras(); ++i) {
    _cameraDvs[i].reset(
        new CameraGeometryDesignVariableContainer(cameraSystem->geometry(i),
                                                  estimateProjection,
                                                  estimateDistortion,
                                                  estimateShutter));
  }

  saveCurrentValues();
}

NCameraSystemDesignVariableContainer::~NCameraSystemDesignVariableContainer() {

}

/// \brief Get all design variables associated with this camera system
void NCameraSystemDesignVariableContainer::getDesignVariables(
    backend::DesignVariable::set_t & dvs) const {

BOOST_FOREACH( const boost::shared_ptr< backend::MappedRotationQuaternion> & q, _q_v_c) {
  dvs.insert(q.get());
}

BOOST_FOREACH( const boost::shared_ptr< backend::MappedEuclideanPoint> & p, _p_v_c) {
  dvs.insert(p.get());
}

}

/// \brief set all intrinsics active
void NCameraSystemDesignVariableContainer::setIntrinsicsActive(bool active) {
BOOST_FOREACH(boost::shared_ptr < CameraGeometryDesignVariableContainer > &cam,
              _cameraDvs)
{
  cam->setActive(active);
}
}

/// \brief set the intrinsics of a specific camera active
void NCameraSystemDesignVariableContainer::setCameraIntrinsicsActive(
  size_t cameraIndex, bool active) {
SM_ASSERT_LT(std::runtime_error, cameraIndex, _cameraDvs.size(),
             "Index out of bounds");
_cameraDvs[cameraIndex]->setActive(active);
}

/// \brief set all extrinsics active
void NCameraSystemDesignVariableContainer::setExtrinsicsActive(bool active) {

setExtrinsicsRotationActive(active);
setExtrinsicsTranslationActive(active);

}

/// \brief set all extrinsics active
void NCameraSystemDesignVariableContainer::setExtrinsicsRotationActive(
  bool active) {
BOOST_FOREACH(boost::shared_ptr < backend::MappedRotationQuaternion > &q,
              _q_v_c)
{
  q->setActive(active);
}
}

/// \brief set all extrinsics active
void NCameraSystemDesignVariableContainer::setExtrinsicsTranslationActive(
  bool active) {
BOOST_FOREACH(boost::shared_ptr < backend::MappedEuclideanPoint > &p, _p_v_c)
{
  p->setActive(active);
}
}

/// \brief set the intrinsics of a specific camera active
void NCameraSystemDesignVariableContainer::setCameraExtrinsicsActive(
  size_t cameraIndex, bool active) {
SM_ASSERT_LT(std::runtime_error, cameraIndex, _q_v_c.size(),
             "Index out of bounds");
_q_v_c[cameraIndex]->setActive(active);
_p_v_c[cameraIndex]->setActive(active);
}

/// \brief set the rotation extrinsics of a specific camera active
void NCameraSystemDesignVariableContainer::setCameraRotationExtrinsicsActive(
  size_t cameraIndex, bool active) {
SM_ASSERT_LT(std::runtime_error, cameraIndex, _q_v_c.size(),
             "Index out of bounds");
_q_v_c[cameraIndex]->setActive(active);
}

/// \brief set the translation extrinsics of a specific camera active
void NCameraSystemDesignVariableContainer::setCameraTranslationExtrinsicsActive(
  size_t cameraIndex, bool active) {
SM_ASSERT_LT(std::runtime_error, cameraIndex, _p_v_c.size(),
             "Index out of bounds");
_p_v_c[cameraIndex]->setActive(active);
}

backend::TransformationExpression NCameraSystemDesignVariableContainer::getT_c_v(
  size_t cameraIndex) {
SM_ASSERT_LT(std::runtime_error, cameraIndex, _T_c_v.size(),
             "Index out of bounds");
return _T_c_v[cameraIndex];
}

/// \brief are the rotation extrinsics of a specific camera active?
bool NCameraSystemDesignVariableContainer::isCameraRotationExtrinsicsActive(
  size_t cameraIndex) const {
SM_ASSERT_LT(std::runtime_error, cameraIndex, _q_v_c.size(),
             "Index out of bounds");
return _q_v_c[cameraIndex]->isActive();
}

/// \brief are the translation extrinsics of a specific camera active?
bool NCameraSystemDesignVariableContainer::isCameraTranslationExtrinsicsActive(
  size_t cameraIndex) const {
SM_ASSERT_LT(std::runtime_error, cameraIndex, _p_v_c.size(),
             "Index out of bounds");
return _p_v_c[cameraIndex]->isActive();
}

/// \brief are the intrinsics of a specific camera active?
bool NCameraSystemDesignVariableContainer::isCameraIntrinsicsActive(
  size_t cameraIndex) const {
SM_ASSERT_LT(std::runtime_error, cameraIndex, _cameraDvs.size(),
             "Index out of bounds");
return _cameraDvs[cameraIndex]->isActive();
}

boost::shared_ptr<CameraGeometryDesignVariableContainer> NCameraSystemDesignVariableContainer::getCameraGeometryDesignVariableContainer(
  size_t cameraIndex) {
SM_ASSERT_LT(std::runtime_error, cameraIndex, _cameraDvs.size(),
             "Index out of bounds");
return _cameraDvs[cameraIndex];
}

boost::shared_ptr<ReprojectionError> NCameraSystemDesignVariableContainer::createReprojectionError(
  MultiFrame & mf, const KeypointIdentifier & kid,
  backend::HomogeneousExpression & p_v) {

boost::shared_ptr<ReprojectionError> re;

boost::shared_ptr<FrameBase> frame = mf.getFrame(kid.cameraIndex);

re = _cameraDvs[kid.cameraIndex]->createReprojectionError(
    frame->keypointBase(kid.keypointIndex).vsY(),
    frame->keypointBase(kid.keypointIndex).vsInvR(),
    _T_c_v[kid.cameraIndex] * p_v);

return re;

}

/// \brief save the current values for this camera system
void NCameraSystemDesignVariableContainer::saveCurrentValues() {

for (size_t i = 0; i < _cameraSystem->numCameras(); ++i) {
  _q_v_c[i]->getParameters(_savedQuaternions[i]);
  _p_v_c[i]->getParameters(_savedEuclidean[i]);
}

for (size_t i = 0; i < _cameraSystem->numCameras(); ++i) {
  _cameraDvs[i]->getParameters(_savedIntrinsics[i]);
}

}

/// \brief restore the values of this camera system
void NCameraSystemDesignVariableContainer::restoreSavedValues() {

for (size_t i = 0; i < _cameraSystem->numCameras(); ++i) {
  _q_v_c[i]->setParameters(_savedQuaternions[i]);
  _p_v_c[i]->setParameters(_savedEuclidean[i]);
}

for (size_t i = 0; i < _cameraSystem->numCameras(); ++i) {
  _cameraDvs[i]->setParameters(_savedIntrinsics[i]);
}

}

NCameraSystem::Ptr NCameraSystemDesignVariableContainer::getCameraSystem() const {
return _cameraSystem;
}

}  // namespace aslam
