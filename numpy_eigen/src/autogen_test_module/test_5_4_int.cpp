#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, 5, 4> test_int_5_4(const Eigen::Matrix<int, 5, 4> & M)
{
	return M;
}
void export_int_5_4()
{
	boost::python::def("test_int_5_4",test_int_5_4);
}

