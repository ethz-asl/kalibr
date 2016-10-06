#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 2, 6> test_double_2_6(const Eigen::Matrix<double, 2, 6> & M)
{
	return M;
}
void export_double_2_6()
{
	boost::python::def("test_double_2_6",test_double_2_6);
}

