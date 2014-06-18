#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, Eigen::Dynamic, 5> test_float_D_5(const Eigen::Matrix<float, Eigen::Dynamic, 5> & M)
{
	return M;
}
void export_float_D_5()
{
	boost::python::def("test_float_D_5",test_float_D_5);
}

