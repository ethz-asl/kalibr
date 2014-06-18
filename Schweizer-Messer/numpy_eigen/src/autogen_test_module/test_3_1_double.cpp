#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 3, 1> test_double_3_1(const Eigen::Matrix<double, 3, 1> & M)
{
	return M;
}
void export_double_3_1()
{
	boost::python::def("test_double_3_1",test_double_3_1);
}

