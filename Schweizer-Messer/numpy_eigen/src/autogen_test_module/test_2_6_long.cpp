#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 2, 6> test_long_2_6(const Eigen::Matrix<boost::int64_t, 2, 6> & M)
{
	return M;
}
void export_long_2_6()
{
	boost::python::def("test_long_2_6",test_long_2_6);
}

