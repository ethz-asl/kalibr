#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <bsplines/BSpline.hpp>
#include <aslam/splines/BSplineDesignVariable.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <sstream>
#include <aslam/cameras.hpp>

using namespace bsplines;
using namespace boost::python;
using namespace aslam::splines;
using namespace aslam::backend;
using namespace aslam::cameras;

void exportBSplineMotionError();
void exportSimpleSplineError();
void exportOptBSplines();
void exportAddQuadraticIntegralExpressionErrorTerms();

template<int D>
void exportBsd()
{
    std::stringstream str;
    str << "BSpline" << D << "DesignVariable";
  
    class_< BSplineDesignVariable<D>, boost::shared_ptr< BSplineDesignVariable<D> > >( str.str().c_str(), init<const BSpline &>() )
        .def("spline", &BSplineDesignVariable<D>::spline, return_value_policy<copy_const_reference>())
        .def("toExpression", &BSplineDesignVariable<D>::toExpression)
        .def("numDesignVariables", &BSplineDesignVariable<D>::numDesignVariables )
        .def("designVariable", &BSplineDesignVariable<D>::designVariable, return_internal_reference<>())
        ;
    
}



BOOST_PYTHON_MODULE(libaslam_splines_python)
{
    exportBsd<1>();
    exportBsd<2>();
    exportBsd<3>();
    exportBsd<4>();
    exportBsd<5>();
    exportBsd<6>();
    exportBsd<7>();
    exportBsd<8>();
    exportBsd<9>();
    exportBsd<10>();
 

    aslam::backend::TransformationExpression (BSplinePoseDesignVariable::*transformationAtTime1)(const aslam::backend::ScalarExpression &) = &BSplinePoseDesignVariable::transformationAtTime;
    aslam::backend::TransformationExpression (BSplinePoseDesignVariable::*transformationAtTime2)(const aslam::backend::ScalarExpression &, double, double) = &BSplinePoseDesignVariable::transformationAtTime;

  
    class_< BSplinePoseDesignVariable, boost::shared_ptr< BSplinePoseDesignVariable > >("BSplinePoseDesignVariable", init<const BSplinePose &>() )
        .def("spline", &BSplinePoseDesignVariable::spline, return_value_policy<copy_const_reference>())
        .def("numDesignVariables", &BSplinePoseDesignVariable::numDesignVariables )
        .def("designVariable", &BSplinePoseDesignVariable::designVariable, return_internal_reference<>())
        .def("transformation", &BSplinePoseDesignVariable::transformation)
        .def("angularVelocityBodyFrame", &BSplinePoseDesignVariable::angularVelocityBodyFrame)
        .def("angularAccelerationBodyFrame", &BSplinePoseDesignVariable::angularAccelerationBodyFrame)
        .def("linearVelocity", &BSplinePoseDesignVariable::linearVelocity)
        .def("linearAcceleration", &BSplinePoseDesignVariable::linearAcceleration)
        .def("position", &BSplinePoseDesignVariable::position)
        .def("orientation", &BSplinePoseDesignVariable::orientation)
        .def("transformationAtTime", transformationAtTime1)
        .def("transformationAtTime", transformationAtTime2);
        ;
    
    

    class_<EuclideanBSplineDesignVariable, boost::shared_ptr< EuclideanBSplineDesignVariable>, bases<BSplineDesignVariable<3> > >("EuclideanBSplineDesignVariable", init<const BSpline &>())
        .def("toEuclideanExpression", &EuclideanBSplineDesignVariable::toEuclideanExpression)
        .def("toEuclidean", &EuclideanBSplineDesignVariable::toEuclidean)
        ;

    exportBSplineMotionError();
    exportSimpleSplineError();


}
