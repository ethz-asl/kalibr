#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 2, 1> test_long_2_1(const Eigen::Matrix<boost::int64_t, 2, 1> & M)
{
	return M;
}
void export_long_2_1()
{
	boost::python::def("test_long_2_1",test_long_2_1);
}

