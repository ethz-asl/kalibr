#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 6, 2> test_int_6_2(const Eigen::Matrix<int, 6, 2> & M)
{
	return M;
}
void export_int_6_2()
{
	boost::python::def("test_int_6_2",test_int_6_2);
}

