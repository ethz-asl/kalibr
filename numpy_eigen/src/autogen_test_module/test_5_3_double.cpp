#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 5, 3> test_double_5_3(const Eigen::Matrix<double, 5, 3> & M)
{
	return M;
}
void export_double_5_3()
{
	boost::python::def("test_double_5_3",test_double_5_3);
}

