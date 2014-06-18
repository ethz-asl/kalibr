#include <numpy_eigen/boost_python_headers.hpp>

#include <aslam/backend/BSplineMotionError.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <aslam/splines/BSplineRSPoseDesignVariable.hpp>

#include <boost/shared_ptr.hpp>



using namespace boost::python;
using namespace aslam::backend;

void exportBSplineMotionError()
{


    class_<BSplineMotionError<aslam::splines::EuclideanBSplineDesignVariable>, boost::shared_ptr<BSplineMotionError<aslam::splines::EuclideanBSplineDesignVariable> >, bases<ErrorTerm> >
        ("BSplineEuclideanMotionError", init<aslam::splines::EuclideanBSplineDesignVariable*, Eigen::MatrixXd >("BSplineEuclideanMotionError(EuclideanBSplineDesignVariable, W)"))
         .def(init<aslam::splines::EuclideanBSplineDesignVariable*, Eigen::MatrixXd, unsigned int >("BSplineGenericMotionError(EuclideanBSplineDesignVariable, W, errorTermOrder)"))
         .def("rhs", &BSplineMotionError<aslam::splines::EuclideanBSplineDesignVariable>::rhs)
         ;


  
    class_<BSplineMotionError<aslam::splines::BSplinePoseDesignVariable>, boost::shared_ptr<BSplineMotionError<aslam::splines::BSplinePoseDesignVariable> >, bases<ErrorTerm> >
        ("BSplineMotionError", init<aslam::splines::BSplinePoseDesignVariable*, Eigen::MatrixXd >("BSplineMotionError(BSplinePoseDesignVariable, W)"))
         .def(init<aslam::splines::BSplinePoseDesignVariable*, Eigen::MatrixXd, unsigned int >("BSplineRSMotionError(BSplinePoseDesignVariable, W, errorTermOrder)"))
         .def("rhs", &BSplineMotionError<aslam::splines::BSplinePoseDesignVariable>::rhs)
         ;

    class_<BSplineMotionError<aslam::splines::BSplineRSPoseDesignVariable>, boost::shared_ptr<BSplineMotionError<aslam::splines::BSplineRSPoseDesignVariable> >, bases<ErrorTerm> >
    ("BSplineRSMotionError", init<aslam::splines::BSplineRSPoseDesignVariable*, Eigen::MatrixXd >("BSplineRSMotionError(BSplinePoseDesignVariable, W)"))
     .def(init<aslam::splines::BSplineRSPoseDesignVariable*, Eigen::MatrixXd, unsigned int >("BSplineRSMotionError(BSplinePoseDesignVariable, W, errorTermOrder)"))
     .def("rhs", &BSplineMotionError<aslam::splines::BSplineRSPoseDesignVariable>::rhs)
     ;
    
    
}
