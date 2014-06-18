#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 5, 5> test_int_5_5(const Eigen::Matrix<int, 5, 5> & M)
{
	return M;
}
void export_int_5_5()
{
	boost::python::def("test_int_5_5",test_int_5_5);
}

