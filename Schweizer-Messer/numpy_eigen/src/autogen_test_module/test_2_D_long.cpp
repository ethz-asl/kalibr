#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 2, Eigen::Dynamic> test_long_2_D(const Eigen::Matrix<boost::int64_t, 2, Eigen::Dynamic> & M)
{
	return M;
}
void export_long_2_D()
{
	boost::python::def("test_long_2_D",test_long_2_D);
}

