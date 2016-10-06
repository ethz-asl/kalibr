#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, Eigen::Dynamic, 6> test_int_D_6(const Eigen::Matrix<int, Eigen::Dynamic, 6> & M)
{
	return M;
}
void export_int_D_6()
{
	boost::python::def("test_int_D_6",test_int_D_6);
}

