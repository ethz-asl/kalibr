#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 6, 4> test_uchar_6_4(const Eigen::Matrix<boost::uint8_t, 6, 4> & M)
{
	return M;
}
void export_uchar_6_4()
{
	boost::python::def("test_uchar_6_4",test_uchar_6_4);
}

