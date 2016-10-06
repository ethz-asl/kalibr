#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 5, 4> test_uchar_5_4(const Eigen::Matrix<boost::uint8_t, 5, 4> & M)
{
	return M;
}
void export_uchar_5_4()
{
	boost::python::def("test_uchar_5_4",test_uchar_5_4);
}

