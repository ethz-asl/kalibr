#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, Eigen::Dynamic, 2> test_long_D_2(const Eigen::Matrix<boost::int64_t, Eigen::Dynamic, 2> & M)
{
	return M;
}
void export_long_D_2()
{
	boost::python::def("test_long_D_2",test_long_D_2);
}

