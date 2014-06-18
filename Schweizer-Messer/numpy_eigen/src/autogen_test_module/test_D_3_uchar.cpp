#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, 3> test_uchar_D_3(const Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, 3> & M)
{
	return M;
}
void export_uchar_D_3()
{
	boost::python::def("test_uchar_D_3",test_uchar_D_3);
}

