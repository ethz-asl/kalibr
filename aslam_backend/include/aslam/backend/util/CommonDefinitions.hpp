#ifndef INCLUDE_ASLAM_BACKEND_COMMON_HPP_
#define INCLUDE_ASLAM_BACKEND_COMMON_HPP_

#include <Eigen/Core>

#include <sm/timing/Timer.hpp>

#if !defined(LIKELY) || !defined(UNLIKELY)
  #if defined(__GNUC__) || defined(__GNUG__)  /* GNU GCC/G++ */
    #define LIKELY(x)    __builtin_expect (!!(x), 1)
    #define UNLIKELY(x)  __builtin_expect (!!(x), 0)
  #else
    #define LIKELY(x)    x
    #define UNLIKELY(x)  x
  #endif
#endif

namespace aslam {
namespace backend {

#ifdef aslam_backend_ENABLE_TIMING
  typedef sm::timing::Timer Timer;
#else
  typedef sm::timing::DummyTimer Timer;
#endif
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> ColumnVectorType;
  typedef Eigen::Matrix<double, 1, Eigen::Dynamic> RowVectorType;

}
}

#endif /* INCLUDE_ASLAM_BACKEND_COMMON_HPP_ */
