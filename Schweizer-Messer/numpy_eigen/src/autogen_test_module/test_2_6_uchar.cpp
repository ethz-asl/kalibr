#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 2, 6> test_uchar_2_6(const Eigen::Matrix<boost::uint8_t, 2, 6> & M)
{
	return M;
}
void export_uchar_2_6()
{
	boost::python::def("test_uchar_2_6",test_uchar_2_6);
}

