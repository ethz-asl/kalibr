#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 5, Eigen::Dynamic> test_long_5_D(const Eigen::Matrix<boost::int64_t, 5, Eigen::Dynamic> & M)
{
	return M;
}
void export_long_5_D()
{
	boost::python::def("test_long_5_D",test_long_5_D);
}

