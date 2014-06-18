#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<int, Eigen::Dynamic, 4> test_int_D_4(const Eigen::Matrix<int, Eigen::Dynamic, 4> & M)
{
	return M;
}
void export_int_D_4()
{
	boost::python::def("test_int_D_4",test_int_D_4);
}

