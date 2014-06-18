#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, Eigen::Dynamic> test_uchar_D_D(const Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, Eigen::Dynamic> & M)
{
	return M;
}
void export_uchar_D_D()
{
	boost::python::def("test_uchar_D_D",test_uchar_D_D);
}

