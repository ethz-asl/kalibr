#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 4, 2> test_int_4_2(const Eigen::Matrix<int, 4, 2> & M)
{
	return M;
}
void export_int_4_2()
{
	boost::python::def("test_int_4_2",test_int_4_2);
}

