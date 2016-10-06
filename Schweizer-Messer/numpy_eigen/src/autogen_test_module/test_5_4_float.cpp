#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 5, 4> test_float_5_4(const Eigen::Matrix<float, 5, 4> & M)
{
	return M;
}
void export_float_5_4()
{
	boost::python::def("test_float_5_4",test_float_5_4);
}

