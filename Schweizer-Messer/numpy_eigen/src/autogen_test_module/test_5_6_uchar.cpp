#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 5, 6> test_uchar_5_6(const Eigen::Matrix<boost::uint8_t, 5, 6> & M)
{
	return M;
}
void export_uchar_5_6()
{
	boost::python::def("test_uchar_5_6",test_uchar_5_6);
}

