#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, 2, 4> test_uchar_2_4(const Eigen::Matrix<boost::uint8_t, 2, 4> & M)
{
	return M;
}
void export_uchar_2_4()
{
	boost::python::def("test_uchar_2_4",test_uchar_2_4);
}

