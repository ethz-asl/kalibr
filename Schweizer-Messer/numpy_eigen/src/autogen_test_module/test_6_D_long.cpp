#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 6, Eigen::Dynamic> test_long_6_D(const Eigen::Matrix<boost::int64_t, 6, Eigen::Dynamic> & M)
{
	return M;
}
void export_long_6_D()
{
	boost::python::def("test_long_6_D",test_long_6_D);
}

