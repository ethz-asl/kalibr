#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 6, 2> test_float_6_2(const Eigen::Matrix<float, 6, 2> & M)
{
	return M;
}
void export_float_6_2()
{
	boost::python::def("test_float_6_2",test_float_6_2);
}

