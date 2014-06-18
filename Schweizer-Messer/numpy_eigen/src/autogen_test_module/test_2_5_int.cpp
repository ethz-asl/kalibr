#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 2, 5> test_int_2_5(const Eigen::Matrix<int, 2, 5> & M)
{
	return M;
}
void export_int_2_5()
{
	boost::python::def("test_int_2_5",test_int_2_5);
}

