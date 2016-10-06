#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, Eigen::Dynamic, 5> test_long_D_5(const Eigen::Matrix<boost::int64_t, Eigen::Dynamic, 5> & M)
{
	return M;
}
void export_long_D_5()
{
	boost::python::def("test_long_D_5",test_long_D_5);
}

