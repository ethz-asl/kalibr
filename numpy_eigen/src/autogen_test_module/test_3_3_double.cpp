#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 3, 3> test_double_3_3(const Eigen::Matrix<double, 3, 3> & M)
{
	return M;
}
void export_double_3_3()
{
	boost::python::def("test_double_3_3",test_double_3_3);
}

