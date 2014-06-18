#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, Eigen::Dynamic, 2> test_float_D_2(const Eigen::Matrix<float, Eigen::Dynamic, 2> & M)
{
	return M;
}
void export_float_D_2()
{
	boost::python::def("test_float_D_2",test_float_D_2);
}

