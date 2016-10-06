#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 5, 1> test_uchar_5_1(const Eigen::Matrix<boost::uint8_t, 5, 1> & M)
{
	return M;
}
void export_uchar_5_1()
{
	boost::python::def("test_uchar_5_1",test_uchar_5_1);
}

