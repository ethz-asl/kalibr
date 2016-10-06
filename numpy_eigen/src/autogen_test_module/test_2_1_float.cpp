#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 2, 1> test_float_2_1(const Eigen::Matrix<float, 2, 1> & M)
{
	return M;
}
void export_float_2_1()
{
	boost::python::def("test_float_2_1",test_float_2_1);
}

