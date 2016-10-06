#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 3, 5> test_long_3_5(const Eigen::Matrix<boost::int64_t, 3, 5> & M)
{
	return M;
}
void export_long_3_5()
{
	boost::python::def("test_long_3_5",test_long_3_5);
}

