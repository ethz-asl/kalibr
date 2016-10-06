#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, Eigen::Dynamic, 3> test_long_D_3(const Eigen::Matrix<boost::int64_t, Eigen::Dynamic, 3> & M)
{
	return M;
}
void export_long_D_3()
{
	boost::python::def("test_long_D_3",test_long_D_3);
}

