#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/kinematics/property_tree.hpp>

void export_kinematics_property_tree()
{
    boost::python::def("transformationFromPropertyTree", &sm::kinematics::transformationFromPropertyTree);
}
