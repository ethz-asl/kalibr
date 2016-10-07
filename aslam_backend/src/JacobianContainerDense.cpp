#include <aslam/backend/JacobianContainerDense.hpp>

namespace aslam {
namespace backend {

// Explicit template instantiation
template class JacobianContainerDense<Eigen::MatrixXd, Eigen::Dynamic>;
template class JacobianContainerDense<Eigen::MatrixXd, 1>;
template class JacobianContainerDense<Eigen::MatrixXd, 2>;
template class JacobianContainerDense<Eigen::MatrixXd&, Eigen::Dynamic>;
template class JacobianContainerDense<Eigen::MatrixXd&, 1>;
template class JacobianContainerDense<Eigen::MatrixXd&, 2>;

} // namespace backend
} // namespace aslam

