#include <bsplines/python/DiffManifoldBSplineExporter.hpp>
#include <aslam/splines/OPTUnitQuaternionBSpline.hpp>
#include <aslam/splines/OPTEuclideanBSpline.hpp>
#include <aslam/splines/OPTEuclideanBSpline.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <vector>

typedef std::vector<aslam::backend::DesignVariable *> DesignVariables;

using namespace bspline_exporter;
using namespace bsplines;

template <typename Spline>
void addOptSplineFunctions (BSplineExporter<Spline> & splineExporter){
  using namespace boost::python;
  splineExporter.getSplineClass()
      .def("getDesignVariables", static_cast<const DesignVariables &(Spline::*)() >(&Spline::getDesignVariables), "Get a list of all design variables", return_value_policy<copy_const_reference>())
      ;
}

void exportOptBSplines() {
  class_<DesignVariables>("DesignVariables")
      .def(boost::python::vector_indexing_suite<DesignVariables>() );

  auto euclidBSpl = BSplineExporter<aslam::splines::OPTBSpline<EuclideanBSpline<>::CONF>::BSpline>::exportEuclideanSpline("OptEuclideanBSpline");
  addOptSplineFunctions(euclidBSpl);
  auto unitQuatBSpl = BSplineExporter<aslam::splines::OPTBSpline<UnitQuaternionBSpline<>::CONF>::BSpline>::exportUnitQuaternionSpline("OptUnitQuaternionBSpline");
  addOptSplineFunctions(unitQuatBSpl);
}
