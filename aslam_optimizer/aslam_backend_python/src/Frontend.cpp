#include <numpy_eigen/boost_python_headers.hpp>



void exportFrame();
void exportDescriptors();

void exportFrontend()
{
  exportFrame();
  exportDescriptors();
}
