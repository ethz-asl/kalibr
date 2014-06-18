#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 5, 1> test_int_5_1(const Eigen::Matrix<int, 5, 1> & M)
{
	return M;
}
void export_int_5_1()
{
	boost::python::def("test_int_5_1",test_int_5_1);
}

