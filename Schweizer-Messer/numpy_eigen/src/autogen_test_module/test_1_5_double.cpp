#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 1, 5> test_double_1_5(const Eigen::Matrix<double, 1, 5> & M)
{
	return M;
}
void export_double_1_5()
{
	boost::python::def("test_double_1_5",test_double_1_5);
}

