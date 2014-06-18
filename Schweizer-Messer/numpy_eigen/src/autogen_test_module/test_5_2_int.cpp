#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 5, 2> test_int_5_2(const Eigen::Matrix<int, 5, 2> & M)
{
	return M;
}
void export_int_5_2()
{
	boost::python::def("test_int_5_2",test_int_5_2);
}

