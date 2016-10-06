#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, Eigen::Dynamic, 5> test_double_D_5(const Eigen::Matrix<double, Eigen::Dynamic, 5> & M)
{
	return M;
}
void export_double_D_5()
{
	boost::python::def("test_double_D_5",test_double_D_5);
}

