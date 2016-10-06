#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 4, 4> test_float_4_4(const Eigen::Matrix<float, 4, 4> & M)
{
	return M;
}
void export_float_4_4()
{
	boost::python::def("test_float_4_4",test_float_4_4);
}

