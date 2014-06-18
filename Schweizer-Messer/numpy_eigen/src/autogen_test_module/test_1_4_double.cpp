#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 1, 4> test_double_1_4(const Eigen::Matrix<double, 1, 4> & M)
{
	return M;
}
void export_double_1_4()
{
	boost::python::def("test_double_1_4",test_double_1_4);
}

