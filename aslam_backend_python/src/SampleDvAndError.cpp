#include <numpy_eigen/boost_python_headers.hpp>

#include <aslam/backend/../../../test/SampleDvAndError.hpp>

#include <boost/shared_ptr.hpp>


using namespace boost::python;
using namespace aslam::backend;

void exportSampleDvAndError()
{

    class_<Point2d, boost::shared_ptr<Point2d>, bases<DesignVariable> >
      ("Point2d", init<const Eigen::Vector2d&>("Point2d(vector2d v): Constructor"))
      ;

    class_<LinearErr, boost::shared_ptr<LinearErr>, bases< aslam::backend::ErrorTermFs<2> > >
      ("LinearErr", init<Point2d*>("LinearErr(Point2d p): Constructor"))
      ;

    class_<LinearErr2, boost::shared_ptr<LinearErr2>, bases< aslam::backend::ErrorTermFs<2> > >
      ("LinearErr2", init<Point2d*,Point2d*>("LinearErr2(Point2d p0, Point2d p1): Constructor"))
      ;

    class_<LinearErr3, boost::shared_ptr<LinearErr3>, bases< aslam::backend::ErrorTermFs<4> > >
      ("LinearErr3", init<Point2d*,Point2d*,Point2d*>("LinearErr3(Point2d p0, Point2d p1, Point2d p2): Constructor"))
      ;

    class_<TestNonSquaredError, boost::shared_ptr<TestNonSquaredError>, bases<ScalarNonSquaredErrorTerm> >
      ("TestNonSquaredError", init<Point2d*, const TestNonSquaredError::grad_t&>("TestNonSquaredError(Point2d p, Vector2d grad): Constructor"))
      .def_readwrite("_p", &TestNonSquaredError::_p)
      .def_readwrite("_grad", &TestNonSquaredError::_grad)
      .def_readwrite("_p2d", &TestNonSquaredError::_p2d)
      ;

}
