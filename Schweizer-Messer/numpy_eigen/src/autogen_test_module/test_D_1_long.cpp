#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, Eigen::Dynamic, 1> test_long_D_1(const Eigen::Matrix<boost::int64_t, Eigen::Dynamic, 1> & M)
{
	return M;
}
void export_long_D_1()
{
	boost::python::def("test_long_D_1",test_long_D_1);
}

