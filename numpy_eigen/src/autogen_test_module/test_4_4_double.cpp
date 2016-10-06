#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 4, 4> test_double_4_4(const Eigen::Matrix<double, 4, 4> & M)
{
	return M;
}
void export_double_4_4()
{
	boost::python::def("test_double_4_4",test_double_4_4);
}

