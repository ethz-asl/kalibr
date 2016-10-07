#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <bsplines/BSpline.hpp>


using namespace bsplines;
using namespace boost::python;

//typedef UniformCubicBSpline<Eigen::Dynamic> UniformCubicBSplineX;


void import_bspline_python();
//void import_rotational_kinematics_python();
void import_bspline_pose_python();
void import_bspline_diff_manifold_python();

BOOST_PYTHON_MODULE(libbsplines_python)
{
	import_bspline_python();
	import_bspline_pose_python();
	import_bspline_diff_manifold_python();

}
