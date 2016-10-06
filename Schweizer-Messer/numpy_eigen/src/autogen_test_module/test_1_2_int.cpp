#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 1, 2> test_int_1_2(const Eigen::Matrix<int, 1, 2> & M)
{
	return M;
}
void export_int_1_2()
{
	boost::python::def("test_int_1_2",test_int_1_2);
}

