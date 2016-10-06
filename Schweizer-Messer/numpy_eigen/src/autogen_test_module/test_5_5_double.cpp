#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 5, 5> test_double_5_5(const Eigen::Matrix<double, 5, 5> & M)
{
	return M;
}
void export_double_5_5()
{
	boost::python::def("test_double_5_5",test_double_5_5);
}

