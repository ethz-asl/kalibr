#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 5, 3> test_float_5_3(const Eigen::Matrix<float, 5, 3> & M)
{
	return M;
}
void export_float_5_3()
{
	boost::python::def("test_float_5_3",test_float_5_3);
}

