#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 6, 6> test_float_6_6(const Eigen::Matrix<float, 6, 6> & M)
{
	return M;
}
void export_float_6_6()
{
	boost::python::def("test_float_6_6",test_float_6_6);
}

