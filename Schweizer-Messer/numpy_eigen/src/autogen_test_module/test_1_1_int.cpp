#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 1, 1> test_int_1_1(const Eigen::Matrix<int, 1, 1> & M)
{
	return M;
}
void export_int_1_1()
{
	boost::python::def("test_int_1_1",test_int_1_1);
}

