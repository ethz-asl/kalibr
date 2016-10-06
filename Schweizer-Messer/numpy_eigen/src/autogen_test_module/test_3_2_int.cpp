#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 3, 2> test_int_3_2(const Eigen::Matrix<int, 3, 2> & M)
{
	return M;
}
void export_int_3_2()
{
	boost::python::def("test_int_3_2",test_int_3_2);
}

