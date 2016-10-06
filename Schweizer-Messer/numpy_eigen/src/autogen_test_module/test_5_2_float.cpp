#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 5, 2> test_float_5_2(const Eigen::Matrix<float, 5, 2> & M)
{
	return M;
}
void export_float_5_2()
{
	boost::python::def("test_float_5_2",test_float_5_2);
}

