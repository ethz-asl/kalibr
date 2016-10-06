#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 1, 1> test_long_1_1(const Eigen::Matrix<boost::int64_t, 1, 1> & M)
{
	return M;
}
void export_long_1_1()
{
	boost::python::def("test_long_1_1",test_long_1_1);
}

