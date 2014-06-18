#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 1, 1> test_uchar_1_1(const Eigen::Matrix<boost::uint8_t, 1, 1> & M)
{
	return M;
}
void export_uchar_1_1()
{
	boost::python::def("test_uchar_1_1",test_uchar_1_1);
}

