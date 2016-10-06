#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 4, 2> test_float_4_2(const Eigen::Matrix<float, 4, 2> & M)
{
	return M;
}
void export_float_4_2()
{
	boost::python::def("test_float_4_2",test_float_4_2);
}

