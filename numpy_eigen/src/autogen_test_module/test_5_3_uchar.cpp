#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 5, 3> test_uchar_5_3(const Eigen::Matrix<boost::uint8_t, 5, 3> & M)
{
	return M;
}
void export_uchar_5_3()
{
	boost::python::def("test_uchar_5_3",test_uchar_5_3);
}

