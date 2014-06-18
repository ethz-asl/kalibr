#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 4, 3> test_uchar_4_3(const Eigen::Matrix<boost::uint8_t, 4, 3> & M)
{
	return M;
}
void export_uchar_4_3()
{
	boost::python::def("test_uchar_4_3",test_uchar_4_3);
}

