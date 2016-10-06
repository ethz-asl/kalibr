#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 6, 1> test_float_6_1(const Eigen::Matrix<float, 6, 1> & M)
{
	return M;
}
void export_float_6_1()
{
	boost::python::def("test_float_6_1",test_float_6_1);
}

