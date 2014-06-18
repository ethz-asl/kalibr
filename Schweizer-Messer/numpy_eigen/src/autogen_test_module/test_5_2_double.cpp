#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 5, 2> test_double_5_2(const Eigen::Matrix<double, 5, 2> & M)
{
	return M;
}
void export_double_5_2()
{
	boost::python::def("test_double_5_2",test_double_5_2);
}

