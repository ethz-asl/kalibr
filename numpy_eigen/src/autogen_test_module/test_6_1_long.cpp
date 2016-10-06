#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 6, 1> test_long_6_1(const Eigen::Matrix<boost::int64_t, 6, 1> & M)
{
	return M;
}
void export_long_6_1()
{
	boost::python::def("test_long_6_1",test_long_6_1);
}

