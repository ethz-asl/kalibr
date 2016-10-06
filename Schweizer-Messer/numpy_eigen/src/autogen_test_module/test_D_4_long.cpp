#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, Eigen::Dynamic, 4> test_long_D_4(const Eigen::Matrix<boost::int64_t, Eigen::Dynamic, 4> & M)
{
	return M;
}
void export_long_D_4()
{
	boost::python::def("test_long_D_4",test_long_D_4);
}

