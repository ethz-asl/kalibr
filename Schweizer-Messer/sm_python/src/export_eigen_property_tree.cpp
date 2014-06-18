#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/eigen/property_tree.hpp>

void export_eigen_property_tree()
{
    boost::python::def("vector3FromPropertyTree", &sm::eigen::vector3FromPropertyTree);
    boost::python::def("quaternionFromPropertyTree", &sm::eigen::quaternionFromPropertyTree);
}
