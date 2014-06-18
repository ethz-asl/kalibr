#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <sm/timing/Timer.hpp>
using namespace boost::python;

//typedef UniformCubicBSpline<Eigen::Dynamic> UniformCubicBSplineX;


void import_rotational_kinematics_python();
void export_rotations();
void export_transformations();
void export_quaternion_algebra();
void export_homogeneous_coordinates();
void exportTransformation();
void exportHomogeneousPoint();
void exportTimestampCorrectors();
void exportPropertyTree();
void exportEigen();
void exportUncertainVector();
void exportMatrixArchive();
void exportLogging();
void exportNsecTime();
void exportRandom();
void export_eigen_property_tree();
void export_kinematics_property_tree();

void printTiming()
{
    sm::timing::Timing::print(std::cout);
}

BOOST_PYTHON_MODULE(libsm_python)
{
    def("printTiming", &printTiming);
  import_rotational_kinematics_python();
  export_rotations();
  export_transformations();
  export_quaternion_algebra();
  export_homogeneous_coordinates();
  exportTransformation();
  exportHomogeneousPoint();
  exportTimestampCorrectors();
  exportPropertyTree();
  exportEigen();
  exportUncertainVector();
  exportMatrixArchive();
  exportLogging();
  exportNsecTime();
  exportRandom();
  export_eigen_property_tree();
  export_kinematics_property_tree();
}
