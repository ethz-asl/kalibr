#ifndef ASLAM_PYTHON_EXPORT_CAMERA_DESIGN_VARIABLE_HPP
#define ASLAM_PYTHON_EXPORT_CAMERA_DESIGN_VARIABLE_HPP
#include <sstream>
#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/python/ExportDesignVariableAdapter.hpp>
#include <aslam/python/ExportBackendExpressions.hpp>
#include <aslam/ExportScalarExpressionNodeKeypointTime.hpp>

namespace aslam {
namespace python {

template<typename C>
void exportCameraDesignVariables(std::string name) {

  using namespace aslam;
  using namespace aslam::python;
  using namespace aslam::backend;

  boost::python::class_<CameraDesignVariable<C>,
      boost::shared_ptr<CameraDesignVariable<C> > >(
      (name + "DesignVariable").c_str(),
      boost::python::init<boost::shared_ptr<C> >())
      .def("euclideanToKeypoint", &CameraDesignVariable<C>::euclideanToKeypoint)
      .def("homogeneousToKeypoint", &CameraDesignVariable<C>::homogeneousToKeypoint)
      .def("setActive", &CameraDesignVariable<C>::setActive)
      .def("getDesignVariables", &aslam::python::getDesignVariables<CameraDesignVariable<C> >)
      .def("projectionDesignVariable", &CameraDesignVariable<C>::projectionDesignVariable)
      .def("distortionDesignVariable", &CameraDesignVariable<C>::distortionDesignVariable)
      .def("shutterDesignVariable", &CameraDesignVariable<C>::shutterDesignVariable)
      .def("camera", &CameraDesignVariable<C>::camera)
      .def("keypointTime", &CameraDesignVariable<C>::keypointTime)
      .def("temporalOffset", &CameraDesignVariable<C>::temporalOffset)
    ;

  aslam::python::exportScalarExpressionNodeKeypointTime<C>(name);
}

template<typename C>
void exportGenericProjectionDesignVariable(std::string name) {

  exportDesignVariableAdapter < C > (name + "DesignVariable");
}

template<typename C>
void exportShutterDesignVariable(std::string name) {

  exportDesignVariableAdapter < C > (name + "DesignVariable");

}

}  // namespace python
}  // namespace aslam

#endif /* ASLAM_PYTHON_EXPORT_CAMERA_DESIGN_VARIABLE_HPP */
