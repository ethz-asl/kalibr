#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, Eigen::Dynamic, 6> test_float_D_6(const Eigen::Matrix<float, Eigen::Dynamic, 6> & M)
{
	return M;
}
void export_float_D_6()
{
	boost::python::def("test_float_D_6",test_float_D_6);
}

