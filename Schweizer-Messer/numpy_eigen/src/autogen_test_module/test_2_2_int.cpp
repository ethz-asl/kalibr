#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 2, 2> test_int_2_2(const Eigen::Matrix<int, 2, 2> & M)
{
	return M;
}
void export_int_2_2()
{
	boost::python::def("test_int_2_2",test_int_2_2);
}

