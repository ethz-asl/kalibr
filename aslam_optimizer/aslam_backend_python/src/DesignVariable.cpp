#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <boost/shared_ptr.hpp>
#include <aslam/Exceptions.hpp>
using namespace boost::python;
using namespace aslam::backend;


void update(DesignVariable * dv, const Eigen::VectorXd & dp)
{
    SM_ASSERT_EQ(aslam::InvalidArgumentException, dp.size(), dv->minimalDimensions(), "The update size must match the number of minimal dimensions");
    dv->update(&dp[0], dp.size());
}

void exportDesignVariable()
{

  class_<DesignVariable, boost::shared_ptr<DesignVariable>, boost::noncopyable >("DesignVariable", no_init)
    /// \brief what is the number of dimensions of the perturbation variable.
    .def("minimalDimensions", &DesignVariable::minimalDimensions)

    /// \brief update the design variable.
    .def("update", &update)

    /// \brief Revert the last state update
    .def("revertUpdate", &DesignVariable::revertUpdate)

    /// \brief is this design variable active in the optimization.
    .def("isActive", &DesignVariable::isActive)

    /// \brief set the active state of this design variable.
    .def("setActive", &DesignVariable::setActive)

    /// \brief should this variable be marginalized in the schur-complement step?
    .def("isMarginalized", &DesignVariable::isMarginalized)
      
    /// \brief should this variable be marginalized in the schur-complement step?
    .def("setMarginalized", &DesignVariable::setMarginalized)

    /// \brief get the block index used in the optimization routine. -1 if not being optimized.
    .def("blockIndex", &DesignVariable::blockIndex)

    /// \brief set the block index used in the optimization routine.
    .def("setBlockIndex", &DesignVariable::setBlockIndex)

    .def("setScaling", &DesignVariable::setScaling)

    .def("scaling", &DesignVariable::scaling)
    ;

}
