#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 4, 5> test_float_4_5(const Eigen::Matrix<float, 4, 5> & M)
{
	return M;
}
void export_float_4_5()
{
	boost::python::def("test_float_4_5",test_float_4_5);
}

