#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, Eigen::Dynamic, 4> test_float_D_4(const Eigen::Matrix<float, Eigen::Dynamic, 4> & M)
{
	return M;
}
void export_float_D_4()
{
	boost::python::def("test_float_D_4",test_float_D_4);
}

