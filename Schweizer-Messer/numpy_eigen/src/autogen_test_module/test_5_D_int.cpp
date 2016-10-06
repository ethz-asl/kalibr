#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 5, Eigen::Dynamic> test_int_5_D(const Eigen::Matrix<int, 5, Eigen::Dynamic> & M)
{
	return M;
}
void export_int_5_D()
{
	boost::python::def("test_int_5_D",test_int_5_D);
}

