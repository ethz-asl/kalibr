#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 6, 5> test_int_6_5(const Eigen::Matrix<int, 6, 5> & M)
{
	return M;
}
void export_int_6_5()
{
	boost::python::def("test_int_6_5",test_int_6_5);
}

