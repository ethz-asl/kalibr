// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>

#include <aslam/GridCalibrationTargetDesignVariableContainer.hpp>
#include <sm/boost/null_deleter.hpp>
#include "helpers.hpp"

void exportGridCalibration() {

  using namespace boost::python;
  using namespace aslam;
  
  class_<GridCalibrationTargetDesignVariableContainer, boost::shared_ptr< GridCalibrationTargetDesignVariableContainer>, boost::noncopyable>
    ( "GridCalibrationTargetDesignVariableContainer",
      init<boost::shared_ptr<cameras::GridCalibrationTargetBase>, bool>("GridCalibrationTargetDesignVariableContainer(target, isEstimationActive)") )
      .def("setPointActive", &GridCalibrationTargetDesignVariableContainer::setPointActive)
      .def("isPointActive", &GridCalibrationTargetDesignVariableContainer::isPointActive)
      .def("getPoint", &GridCalibrationTargetDesignVariableContainer::getPoint)
      .def("getTarget", &GridCalibrationTargetDesignVariableContainer::getTarget)
      .def("getDesignVariables", &getDesignVariablesWrap<GridCalibrationTargetDesignVariableContainer>);

}
