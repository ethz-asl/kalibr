#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 2, 2> test_float_2_2(const Eigen::Matrix<float, 2, 2> & M)
{
	return M;
}
void export_float_2_2()
{
	boost::python::def("test_float_2_2",test_float_2_2);
}

