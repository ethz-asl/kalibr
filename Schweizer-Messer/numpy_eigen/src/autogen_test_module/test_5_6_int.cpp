#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 5, 6> test_int_5_6(const Eigen::Matrix<int, 5, 6> & M)
{
	return M;
}
void export_int_5_6()
{
	boost::python::def("test_int_5_6",test_int_5_6);
}

