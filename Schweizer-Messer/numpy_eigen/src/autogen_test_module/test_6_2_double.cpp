#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 6, 2> test_double_6_2(const Eigen::Matrix<double, 6, 2> & M)
{
	return M;
}
void export_double_6_2()
{
	boost::python::def("test_double_6_2",test_double_6_2);
}

