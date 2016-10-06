#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 1, 3> test_float_1_3(const Eigen::Matrix<float, 1, 3> & M)
{
	return M;
}
void export_float_1_3()
{
	boost::python::def("test_float_1_3",test_float_1_3);
}

