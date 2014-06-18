#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/OptimizerOptions.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <boost/shared_ptr.hpp>
#include <aslam/backend/LinearSystemSolver.hpp>
#include <aslam/backend/TrustRegionPolicy.hpp>
void exportOptimizerOptions()
{
  using namespace boost::python;
  using namespace aslam::backend;
  class_<OptimizerOptions>("OptimizerOptions", init<>())
    .def_readwrite("convergenceDeltaJ",&OptimizerOptions::convergenceDeltaJ)
    .def_readwrite("convergenceDeltaX",&OptimizerOptions::convergenceDeltaX)
    .def_readwrite("levenbergMarquardtLambdaInit",&OptimizerOptions::levenbergMarquardtLambdaInit)   
    .def_readwrite("levenbergMarquardtLambdaBeta", &OptimizerOptions::levenbergMarquardtLambdaBeta)
    .def_readwrite("levenbergMarquardtLambdaP", &OptimizerOptions::levenbergMarquardtLambdaP)
    .def_readwrite("levenbergMarquardtLambdaMuInit",&OptimizerOptions::levenbergMarquardtLambdaMuInit)
    .def_readwrite("levenbergMarquardtEstimateLambdaScale",&OptimizerOptions::levenbergMarquardtEstimateLambdaScale)
    .def_readwrite("doLevenbergMarquardt",&OptimizerOptions::doLevenbergMarquardt) 
    .def_readwrite("doSchurComplement",&OptimizerOptions::doSchurComplement)
    .def_readwrite("maxIterations",&OptimizerOptions::maxIterations)
    .def_readwrite("verbose",&OptimizerOptions::verbose)
    .def_readwrite("linearSolver",&OptimizerOptions::linearSolver)
    .def_readwrite("resetSolverEveryIteration", &OptimizerOptions::resetSolverEveryIteration)
    ;

  using namespace boost::python;
  using namespace aslam::backend;
  class_<Optimizer2Options>("Optimizer2Options", init<>())
    .def_readwrite("convergenceDeltaJ",&Optimizer2Options::convergenceDeltaJ)
    .def_readwrite("convergenceDeltaX",&Optimizer2Options::convergenceDeltaX)

      //.def_readwrite("doLevenbergMarquardt",&Optimizer2Options::doLevenbergMarquardt) 
    .def_readwrite("doSchurComplement",&Optimizer2Options::doSchurComplement)
    .def_readwrite("maxIterations",&Optimizer2Options::maxIterations)
    .def_readwrite("verbose",&Optimizer2Options::verbose)
    .def_readwrite("linearSolver",&Optimizer2Options::linearSystemSolver)
    .def_readwrite("nThreads", &Optimizer2Options::nThreads)
      .def_readwrite("trustRegionPolicy", &Optimizer2Options::trustRegionPolicy)
    ;


}

