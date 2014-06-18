#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 4, 5> test_long_4_5(const Eigen::Matrix<boost::int64_t, 4, 5> & M)
{
	return M;
}
void export_long_4_5()
{
	boost::python::def("test_long_4_5",test_long_4_5);
}

