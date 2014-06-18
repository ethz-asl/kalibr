#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 4, 4> test_int_4_4(const Eigen::Matrix<int, 4, 4> & M)
{
	return M;
}
void export_int_4_4()
{
	boost::python::def("test_int_4_4",test_int_4_4);
}

