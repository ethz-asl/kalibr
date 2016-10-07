/*
 * BasisMatrixFunctor.hpp
 *
 *  Created on: Apr 17, 2013
 *      Author: hannes
 */

#ifndef BASISMATRIXFUNCTOR_HPP_
#define BASISMATRIXFUNCTOR_HPP_
#include "Eigen/Core"

namespace aslam {
namespace backend {
namespace internal {
struct BasisMatrixNullaryFunctor {
 private:
  Eigen::DenseIndex _cwiseIndex, _i, _j;
 public:
  typedef int result_type;
  inline BasisMatrixNullaryFunctor(const Eigen::DenseIndex& cwiseIndex, const Eigen::DenseIndex& i, const Eigen::DenseIndex& j)
      : _cwiseIndex(cwiseIndex),
        _i(i),
        _j(j) {
  }
  inline result_type operator()(Eigen::DenseIndex& index) const {
    return (_cwiseIndex == index) ? 1 : 0;
  }

  inline result_type operator()(Eigen::DenseIndex& row, Eigen::DenseIndex& col) const {
    return (_i == row && _j == col) ? 1 : 0;
  }
};
}
}
}
namespace Eigen {
namespace internal {
template<>
struct functor_traits<aslam::backend::internal::BasisMatrixNullaryFunctor> {
  enum {
    Cost = 1,
    PacketAccess = false,
    IsRepeatable = true
  };
};
}
}

#endif /* BASISMATRIXFUNCTOR_HPP_ */
