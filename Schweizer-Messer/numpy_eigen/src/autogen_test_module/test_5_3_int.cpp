#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 5, 3> test_int_5_3(const Eigen::Matrix<int, 5, 3> & M)
{
	return M;
}
void export_int_5_3()
{
	boost::python::def("test_int_5_3",test_int_5_3);
}

