#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 4, 6> test_double_4_6(const Eigen::Matrix<double, 4, 6> & M)
{
	return M;
}
void export_double_4_6()
{
	boost::python::def("test_double_4_6",test_double_4_6);
}

