#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<double, 4, 2> test_double_4_2(const Eigen::Matrix<double, 4, 2> & M)
{
	return M;
}
void export_double_4_2()
{
	boost::python::def("test_double_4_2",test_double_4_2);
}

