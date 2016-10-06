#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 2, 1> test_double_2_1(const Eigen::Matrix<double, 2, 1> & M)
{
	return M;
}
void export_double_2_1()
{
	boost::python::def("test_double_2_1",test_double_2_1);
}

