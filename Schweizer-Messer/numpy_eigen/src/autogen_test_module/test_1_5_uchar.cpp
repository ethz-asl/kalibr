#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 1, 5> test_uchar_1_5(const Eigen::Matrix<boost::uint8_t, 1, 5> & M)
{
	return M;
}
void export_uchar_1_5()
{
	boost::python::def("test_uchar_1_5",test_uchar_1_5);
}

