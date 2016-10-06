#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 3, 3> test_float_3_3(const Eigen::Matrix<float, 3, 3> & M)
{
	return M;
}
void export_float_3_3()
{
	boost::python::def("test_float_3_3",test_float_3_3);
}

