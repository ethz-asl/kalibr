#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 4, 1> test_long_4_1(const Eigen::Matrix<boost::int64_t, 4, 1> & M)
{
	return M;
}
void export_long_4_1()
{
	boost::python::def("test_long_4_1",test_long_4_1);
}

