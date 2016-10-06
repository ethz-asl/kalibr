#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 4, 2> test_long_4_2(const Eigen::Matrix<boost::int64_t, 4, 2> & M)
{
	return M;
}
void export_long_4_2()
{
	boost::python::def("test_long_4_2",test_long_4_2);
}

