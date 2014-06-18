#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, Eigen::Dynamic, 6> test_long_D_6(const Eigen::Matrix<boost::int64_t, Eigen::Dynamic, 6> & M)
{
	return M;
}
void export_long_D_6()
{
	boost::python::def("test_long_D_6",test_long_D_6);
}

