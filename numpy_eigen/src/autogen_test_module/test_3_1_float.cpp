#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 3, 1> test_float_3_1(const Eigen::Matrix<float, 3, 1> & M)
{
	return M;
}
void export_float_3_1()
{
	boost::python::def("test_float_3_1",test_float_3_1);
}

