#include <aslam/backend/JacobianContainerSparse.hpp>

namespace aslam {
namespace backend {

// Explicit template instantiation
template class JacobianContainerSparse<Eigen::Dynamic>;
template class JacobianContainerSparse<1>;
template class JacobianContainerSparse<2>;

} // namespace backend
} // namespace aslam
