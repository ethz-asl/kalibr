#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 4, 5> test_uchar_4_5(const Eigen::Matrix<boost::uint8_t, 4, 5> & M)
{
	return M;
}
void export_uchar_4_5()
{
	boost::python::def("test_uchar_4_5",test_uchar_4_5);
}

