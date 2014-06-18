#include <sm/eigen/random.hpp>
#include <sm/random.hpp>

namespace sm {
namespace eigen {


Eigen::VectorXd randn(unsigned dim) {
  Eigen::VectorXd v(dim);
  for(unsigned i = 0; i < dim; ++i) {
    v[i] = sm::random::randn();
  }
  return v;
}

} // namespace eigen
} // namespace sm
