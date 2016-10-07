#include <numpy_eigen/boost_python_headers.hpp>
#include <Eigen/Core>
#include <aslam/backend/DesignVariableTimePair.hpp>

#include <aslam/Exceptions.hpp>
using namespace boost::python;
using namespace aslam::backend;

Eigen::MatrixXd getDvParameter(DesignVariableTimePair* dvtp)
{
  Eigen::MatrixXd v;
  dvtp->dv->getParameters(v);
  return v;
}

void exportDesignVariableTimePair()
{

  class_<DesignVariableTimePair>("DesignVariableTimePair", no_init)
    /// \brief pointer to the design varibale
    .def("getDvParameter", getDvParameter)

    /// \brief the time
    .def_readonly("t", &DesignVariableTimePair::t)
    ;

}
