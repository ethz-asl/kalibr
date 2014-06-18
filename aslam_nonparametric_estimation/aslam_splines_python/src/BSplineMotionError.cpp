#include <numpy_eigen/boost_python_headers.hpp>

#include <aslam/backend/BSplineMotionError.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <aslam/backend/BSplineMotionErrorFactory.hpp>
#include <boost/shared_ptr.hpp>
#include <aslam/splines/BSplineDesignVariable.hpp>



using namespace boost::python;
using namespace aslam::backend;

void exportBSplineMotionError()
{

  def("addMotionErrorTerms", &addMotionErrorTerms<aslam::splines::EuclideanBSplineDesignVariable>, "void addMotionErrorTerms( OptimizationProblemBase, SplineDv, W, errorTermOrder)");
  def("addMotionErrorTerms", &addMotionErrorTerms<aslam::splines::BSplinePoseDesignVariable>, "void addMotionErrorTerms( OptimizationProblemBase, SplineDv, W, errorTermOrder)");
  def("addMotionErrorTerms", &addMotionErrorTerms<aslam::splines::BSplineDesignVariable<1> >, "void addMotionErrorTerms( OptimizationProblemBase, SplineDv, W, errorTermOrder)");
  def("addMotionErrorTerms", &addMotionErrorTerms<aslam::splines::BSplineDesignVariable<2> >, "void addMotionErrorTerms( OptimizationProblemBase, SplineDv, W, errorTermOrder)");
  def("addMotionErrorTerms", &addMotionErrorTerms<aslam::splines::BSplineDesignVariable<3> >, "void addMotionErrorTerms( OptimizationProblemBase, SplineDv, W, errorTermOrder)");


    class_<BSplineMotionError<aslam::splines::EuclideanBSplineDesignVariable>, boost::shared_ptr<BSplineMotionError<aslam::splines::EuclideanBSplineDesignVariable> >, bases<ErrorTerm> >
        ("BSplineEuclideanMotionError", init<aslam::splines::EuclideanBSplineDesignVariable*, Eigen::MatrixXd >("BSplineEuclideanMotionError(EuclideanBSplineDesignVariable, W)"))
         .def(init<aslam::splines::EuclideanBSplineDesignVariable*, Eigen::MatrixXd, unsigned int >("BSplineGenericMotionError(EuclideanBSplineDesignVariable, W, errorTermOrder)"))
         ;
    using namespace aslam::splines;
    class_<BSplineMotionError< BSplineDesignVariable<1> >, boost::shared_ptr<BSplineMotionError< BSplineDesignVariable<1> > >, bases<ErrorTerm> >
        ("BSpline1MotionError", init<BSplineDesignVariable<1> *, Eigen::MatrixXd >("BSpline1MotionError(BSpline1DesignVariable, W)"))
         .def(init<aslam::splines::BSplineDesignVariable<1>*, Eigen::MatrixXd, unsigned int >("BSpline1MotionError(BSpline1DesignVariable, W, errorTermOrder)"))
         ;
    class_<BSplineMotionError< BSplineDesignVariable<2> >, boost::shared_ptr<BSplineMotionError< BSplineDesignVariable<2> > >, bases<ErrorTerm> >
        ("BSpline2MotionError", init<BSplineDesignVariable<2> *, Eigen::MatrixXd >("BSpline2MotionError(BSpline2DesignVariable, W)"))
         .def(init<aslam::splines::BSplineDesignVariable<2>*, Eigen::MatrixXd, unsigned int >("BSpline2MotionError(BSpline2DesignVariable, W, errorTermOrder)"))
         ;

    class_<BSplineMotionError< BSplineDesignVariable<3> >, boost::shared_ptr<BSplineMotionError< BSplineDesignVariable<3> > >, bases<ErrorTerm> >
        ("BSpline3MotionError", init<BSplineDesignVariable<3> *, Eigen::MatrixXd >("BSpline3MotionError(BSpline3DesignVariable, W)"))
         .def(init<aslam::splines::BSplineDesignVariable<3>*, Eigen::MatrixXd, unsigned int >("BSpline3MotionError(BSpline3DesignVariable, W, errorTermOrder)"))
         ;



  
    class_<BSplineMotionError<aslam::splines::BSplinePoseDesignVariable>, boost::shared_ptr<BSplineMotionError<aslam::splines::BSplinePoseDesignVariable> >, bases<ErrorTerm> >
        ("BSplineMotionError", init<aslam::splines::BSplinePoseDesignVariable*, Eigen::MatrixXd >("BSplineMotionError(BSplinePoseDesignVariable, W)"))
         .def(init<aslam::splines::BSplinePoseDesignVariable*, Eigen::MatrixXd, unsigned int >("BSplineMotionError(BSplinePoseDesignVariable, W, errorTermOrder)"))
         ;
    
    
}
