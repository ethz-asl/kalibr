#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 3, 1> test_int_3_1(const Eigen::Matrix<int, 3, 1> & M)
{
	return M;
}
void export_int_3_1()
{
	boost::python::def("test_int_3_1",test_int_3_1);
}

