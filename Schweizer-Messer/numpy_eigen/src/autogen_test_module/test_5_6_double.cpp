#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 5, 6> test_double_5_6(const Eigen::Matrix<double, 5, 6> & M)
{
	return M;
}
void export_double_5_6()
{
	boost::python::def("test_double_5_6",test_double_5_6);
}

