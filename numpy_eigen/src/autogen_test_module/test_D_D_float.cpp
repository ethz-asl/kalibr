#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> test_float_D_D(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> & M)
{
	return M;
}
void export_float_D_D()
{
	boost::python::def("test_float_D_D",test_float_D_D);
}

