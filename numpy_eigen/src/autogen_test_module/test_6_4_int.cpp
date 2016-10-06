#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 6, 4> test_int_6_4(const Eigen::Matrix<int, 6, 4> & M)
{
	return M;
}
void export_int_6_4()
{
	boost::python::def("test_int_6_4",test_int_6_4);
}

