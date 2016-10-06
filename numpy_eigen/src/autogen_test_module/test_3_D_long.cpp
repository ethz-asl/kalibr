#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 3, Eigen::Dynamic> test_long_3_D(const Eigen::Matrix<boost::int64_t, 3, Eigen::Dynamic> & M)
{
	return M;
}
void export_long_3_D()
{
	boost::python::def("test_long_3_D",test_long_3_D);
}

