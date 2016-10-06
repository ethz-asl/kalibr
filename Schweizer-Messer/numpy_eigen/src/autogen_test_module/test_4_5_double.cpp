#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 4, 5> test_double_4_5(const Eigen::Matrix<double, 4, 5> & M)
{
	return M;
}
void export_double_4_5()
{
	boost::python::def("test_double_4_5",test_double_4_5);
}

