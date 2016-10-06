#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 6, 6> test_uchar_6_6(const Eigen::Matrix<boost::uint8_t, 6, 6> & M)
{
	return M;
}
void export_uchar_6_6()
{
	boost::python::def("test_uchar_6_6",test_uchar_6_6);
}

