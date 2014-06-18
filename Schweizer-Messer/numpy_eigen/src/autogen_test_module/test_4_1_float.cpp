#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 4, 1> test_float_4_1(const Eigen::Matrix<float, 4, 1> & M)
{
	return M;
}
void export_float_4_1()
{
	boost::python::def("test_float_4_1",test_float_4_1);
}

