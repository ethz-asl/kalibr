#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 2, 5> test_float_2_5(const Eigen::Matrix<float, 2, 5> & M)
{
	return M;
}
void export_float_2_5()
{
	boost::python::def("test_float_2_5",test_float_2_5);
}

