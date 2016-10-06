#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 1, Eigen::Dynamic> test_uchar_1_D(const Eigen::Matrix<boost::uint8_t, 1, Eigen::Dynamic> & M)
{
	return M;
}
void export_uchar_1_D()
{
	boost::python::def("test_uchar_1_D",test_uchar_1_D);
}

