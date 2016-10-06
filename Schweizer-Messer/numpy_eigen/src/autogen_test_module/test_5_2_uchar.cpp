#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 5, 2> test_uchar_5_2(const Eigen::Matrix<boost::uint8_t, 5, 2> & M)
{
	return M;
}
void export_uchar_5_2()
{
	boost::python::def("test_uchar_5_2",test_uchar_5_2);
}

