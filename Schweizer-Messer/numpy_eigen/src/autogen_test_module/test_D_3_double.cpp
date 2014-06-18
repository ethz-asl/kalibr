#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, Eigen::Dynamic, 3> test_double_D_3(const Eigen::Matrix<double, Eigen::Dynamic, 3> & M)
{
	return M;
}
void export_double_D_3()
{
	boost::python::def("test_double_D_3",test_double_D_3);
}

