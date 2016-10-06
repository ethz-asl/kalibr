#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> test_int_D_D(const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> & M)
{
	return M;
}
void export_int_D_D()
{
	boost::python::def("test_int_D_D",test_int_D_D);
}

