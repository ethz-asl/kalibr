#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 2, 6> test_int_2_6(const Eigen::Matrix<int, 2, 6> & M)
{
	return M;
}
void export_int_2_6()
{
	boost::python::def("test_int_2_6",test_int_2_6);
}

