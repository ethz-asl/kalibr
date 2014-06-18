#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, Eigen::Dynamic, 3> test_float_D_3(const Eigen::Matrix<float, Eigen::Dynamic, 3> & M)
{
	return M;
}
void export_float_D_3()
{
	boost::python::def("test_float_D_3",test_float_D_3);
}

