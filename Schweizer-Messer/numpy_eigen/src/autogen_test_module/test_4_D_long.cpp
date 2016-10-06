#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 4, Eigen::Dynamic> test_long_4_D(const Eigen::Matrix<boost::int64_t, 4, Eigen::Dynamic> & M)
{
	return M;
}
void export_long_4_D()
{
	boost::python::def("test_long_4_D",test_long_4_D);
}

