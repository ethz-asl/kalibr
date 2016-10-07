#include <bsplines/python/DiffManifoldBSplineExporter.hpp>

using namespace bspline_exporter;
using namespace bsplines;
using namespace boost::python;

void import_bspline_diff_manifold_python()
{
	boost::python::to_python_converter<eigenTools::DynamicOrTemplateInt<Eigen::Dynamic>, DynamicOrTemplateInt_to_python_int>();

	{
		typedef typename EuclideanBSpline<>::TYPE::TimePolicy TimePolicy;
		typedef DeltaUniformKnotGenerator<TimePolicy> KnotGenerator;
		class_<KnotGenerator>("DeltaUniformKnotGenerator", init<int, typename TimePolicy::time_t, typename TimePolicy::duration_t, typename TimePolicy::time_t, bool>())
				.def("extendBeyondTime", &KnotGenerator::extendBeyondTime)
				;
	}

	BSplineExporter<EuclideanBSpline<>::TYPE>::exportEuclideanSpline("EuclideanBSpline");
	BSplineExporter<UnitQuaternionBSpline<>::TYPE>::exportUnitQuaternionSpline("UnitQuaternionBSpline");
}
