#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 3, 4> test_long_3_4(const Eigen::Matrix<boost::int64_t, 3, 4> & M)
{
	return M;
}
void export_long_3_4()
{
	boost::python::def("test_long_3_4",test_long_3_4);
}

