// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/NCameraSystemDesignVariableContainer.hpp>
#include <sm/python/stl_converters.hpp>
#include <aslam/ReprojectionError.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include "helpers.hpp"

void exportNCameraSystemDvc() {
  using namespace boost::python;
  using namespace aslam;

  class_<NCameraSystemDesignVariableContainer,
         boost::shared_ptr<NCameraSystemDesignVariableContainer>,
         boost::noncopyable>("NCameraSystemDesignVariableContainer",
                             init<NCameraSystem::Ptr, bool, bool, bool, bool, bool>(
                                 "NCameraSystemDesignVariableContainer(NCameraSystem::Ptr cameraSystem, bool estimateExtrinsicsRotation, bool estimateExtrinsicsTranslation, bool estimateProjection, bool estimateDistortion, bool estimateShutter)"))
      .def("getDesignVariables",
           &getDesignVariablesWrap<NCameraSystemDesignVariableContainer>)
      .def("setIntrinsicsActive",
           &NCameraSystemDesignVariableContainer::setIntrinsicsActive)
      .def("setCameraIntrinsicsActive",
           &NCameraSystemDesignVariableContainer::setCameraIntrinsicsActive)
      .def("setExtrinsicsActive",
           &NCameraSystemDesignVariableContainer::setExtrinsicsActive)
      .def("setCameraExtrinsicsActive",
           &NCameraSystemDesignVariableContainer::setCameraExtrinsicsActive)
      .def("setExtrinsicsRotationActive",
           &NCameraSystemDesignVariableContainer::setExtrinsicsRotationActive)
      .def("setExtrinsicsTranslationActive",
           &NCameraSystemDesignVariableContainer::setExtrinsicsTranslationActive)
      .def("setCameraRotationExtrinsicsActive",
           &NCameraSystemDesignVariableContainer::setCameraRotationExtrinsicsActive)
      
      .def("setCameraTranslationExtrinsicsActive",
           &NCameraSystemDesignVariableContainer::setCameraTranslationExtrinsicsActive)
      
      .def("isCameraRotationExtrinsicsActive",
           &NCameraSystemDesignVariableContainer::isCameraRotationExtrinsicsActive)
      
      .def("isCameraTranslationExtrinsicsActive",
           &NCameraSystemDesignVariableContainer::isCameraTranslationExtrinsicsActive)
      
      .def("isCameraIntrinsicsActive",
           &NCameraSystemDesignVariableContainer::isCameraIntrinsicsActive)
      .def("createReprojectionError",
           &NCameraSystemDesignVariableContainer::createReprojectionError)
      .def("getT_c_v", &NCameraSystemDesignVariableContainer::getT_c_v)
      .def("getCameraGeometryDesignVariableContainer",
           &NCameraSystemDesignVariableContainer::getCameraGeometryDesignVariableContainer)
      .def("saveCurrentValues",
           &NCameraSystemDesignVariableContainer::saveCurrentValues)
      .def("restoreSavedValues",
           &NCameraSystemDesignVariableContainer::restoreSavedValues);
}
