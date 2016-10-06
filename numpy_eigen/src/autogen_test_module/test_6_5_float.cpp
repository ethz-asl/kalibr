#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 6, 5> test_float_6_5(const Eigen::Matrix<float, 6, 5> & M)
{
	return M;
}
void export_float_6_5()
{
	boost::python::def("test_float_6_5",test_float_6_5);
}

