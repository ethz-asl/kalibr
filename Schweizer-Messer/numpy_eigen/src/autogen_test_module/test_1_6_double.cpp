#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 1, 6> test_double_1_6(const Eigen::Matrix<double, 1, 6> & M)
{
	return M;
}
void export_double_1_6()
{
	boost::python::def("test_double_1_6",test_double_1_6);
}

