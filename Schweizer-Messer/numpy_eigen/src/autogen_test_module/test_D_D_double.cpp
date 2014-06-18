#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> test_double_D_D(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & M)
{
	return M;
}
void export_double_D_D()
{
	boost::python::def("test_double_D_D",test_double_D_D);
}

