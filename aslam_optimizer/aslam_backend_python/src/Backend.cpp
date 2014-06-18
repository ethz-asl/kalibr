#include <numpy_eigen/boost_python_headers.hpp>


void exportOptimizer();
void exportOptimizerOptions();
void exportOptimizationProblem();
void exportDesignVariable();
void exportErrorTerm();
//void exportBSplineMotionError();
void exportMEstimators();
void exportJacobianContainer();
void exportBackendExpressions();
//void exportErrorTermTransformation();

void exportBackend()
{
  exportOptimizer();
  exportOptimizerOptions();
  exportOptimizationProblem();
  exportDesignVariable();
  exportErrorTerm();
  //exportBSplineMotionError();
  exportMEstimators();
  exportJacobianContainer();
  exportBackendExpressions();
  //exportErrorTermTransformation();
}

