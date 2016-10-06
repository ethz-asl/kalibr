#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 6, 3> test_double_6_3(const Eigen::Matrix<double, 6, 3> & M)
{
	return M;
}
void export_double_6_3()
{
	boost::python::def("test_double_6_3",test_double_6_3);
}

