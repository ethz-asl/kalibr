#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 3, 5> test_int_3_5(const Eigen::Matrix<int, 3, 5> & M)
{
	return M;
}
void export_int_3_5()
{
	boost::python::def("test_int_3_5",test_int_3_5);
}

