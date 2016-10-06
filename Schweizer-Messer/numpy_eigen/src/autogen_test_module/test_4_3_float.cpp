#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 4, 3> test_float_4_3(const Eigen::Matrix<float, 4, 3> & M)
{
	return M;
}
void export_float_4_3()
{
	boost::python::def("test_float_4_3",test_float_4_3);
}

