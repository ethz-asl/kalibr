#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 2, 3> test_int_2_3(const Eigen::Matrix<int, 2, 3> & M)
{
	return M;
}
void export_int_2_3()
{
	boost::python::def("test_int_2_3",test_int_2_3);
}

