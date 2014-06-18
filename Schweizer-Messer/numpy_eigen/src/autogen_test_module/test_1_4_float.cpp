#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 1, 4> test_float_1_4(const Eigen::Matrix<float, 1, 4> & M)
{
	return M;
}
void export_float_1_4()
{
	boost::python::def("test_float_1_4",test_float_1_4);
}

