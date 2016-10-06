#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 1, 3> test_uchar_1_3(const Eigen::Matrix<boost::uint8_t, 1, 3> & M)
{
	return M;
}
void export_uchar_1_3()
{
	boost::python::def("test_uchar_1_3",test_uchar_1_3);
}

