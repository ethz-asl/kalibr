#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 6, Eigen::Dynamic> test_uchar_6_D(const Eigen::Matrix<boost::uint8_t, 6, Eigen::Dynamic> & M)
{
	return M;
}
void export_uchar_6_D()
{
	boost::python::def("test_uchar_6_D",test_uchar_6_D);
}

