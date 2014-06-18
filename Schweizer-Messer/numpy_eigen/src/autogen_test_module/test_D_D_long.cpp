#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, Eigen::Dynamic, Eigen::Dynamic> test_long_D_D(const Eigen::Matrix<boost::int64_t, Eigen::Dynamic, Eigen::Dynamic> & M)
{
	return M;
}
void export_long_D_D()
{
	boost::python::def("test_long_D_D",test_long_D_D);
}

