#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 1, 5> test_int_1_5(const Eigen::Matrix<int, 1, 5> & M)
{
	return M;
}
void export_int_1_5()
{
	boost::python::def("test_int_1_5",test_int_1_5);
}

