#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 4, Eigen::Dynamic> test_double_4_D(const Eigen::Matrix<double, 4, Eigen::Dynamic> & M)
{
	return M;
}
void export_double_4_D()
{
	boost::python::def("test_double_4_D",test_double_4_D);
}

