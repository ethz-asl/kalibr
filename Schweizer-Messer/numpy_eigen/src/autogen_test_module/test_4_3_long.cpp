#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 4, 3> test_long_4_3(const Eigen::Matrix<boost::int64_t, 4, 3> & M)
{
	return M;
}
void export_long_4_3()
{
	boost::python::def("test_long_4_3",test_long_4_3);
}

