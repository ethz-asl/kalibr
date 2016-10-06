#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 3, 3> test_long_3_3(const Eigen::Matrix<boost::int64_t, 3, 3> & M)
{
	return M;
}
void export_long_3_3()
{
	boost::python::def("test_long_3_3",test_long_3_3);
}

