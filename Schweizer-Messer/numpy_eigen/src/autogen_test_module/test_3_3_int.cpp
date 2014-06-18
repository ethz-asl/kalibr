#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 3, 3> test_int_3_3(const Eigen::Matrix<int, 3, 3> & M)
{
	return M;
}
void export_int_3_3()
{
	boost::python::def("test_int_3_3",test_int_3_3);
}

