// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>

void exportBackend();
void exportCompressedColumnMatrix() ;
void exportLinearSystemSolver();
void exportErrorTermTransformation();
void exportL1Regularizer();
void exportTrustRegionPolicies();
void exportSBM();
void exportErrorTermDs();
void exportDesignVariableTimePair();
void exportSampleDvAndError();
void exportScalarNonSquaredErrorTerm();

// The title of this library must match exactly
BOOST_PYTHON_MODULE(libaslam_backend_python)
{
  // fill this in with boost::python export code
  exportBackend();
  exportCompressedColumnMatrix();
  exportLinearSystemSolver();
  exportErrorTermTransformation();
  exportTrustRegionPolicies();
  exportSBM();
  exportErrorTermDs();
  exportScalarNonSquaredErrorTerm();
  exportL1Regularizer();
  exportDesignVariableTimePair();
  exportSampleDvAndError();
}
