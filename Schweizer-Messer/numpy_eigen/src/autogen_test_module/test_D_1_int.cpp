#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, Eigen::Dynamic, 1> test_int_D_1(const Eigen::Matrix<int, Eigen::Dynamic, 1> & M)
{
	return M;
}
void export_int_D_1()
{
	boost::python::def("test_int_D_1",test_int_D_1);
}

