#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 6, 5> test_double_6_5(const Eigen::Matrix<double, 6, 5> & M)
{
	return M;
}
void export_double_6_5()
{
	boost::python::def("test_double_6_5",test_double_6_5);
}

