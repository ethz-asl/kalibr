#ifndef ASLAM_PER_ITERATION_CALLBACK_HPP
#define ASLAM_PER_ITERATION_CALLBACK_HPP

#include <math.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <aslam/backend/ErrorTerm.hpp>

#include <vector>

namespace aslam {
namespace backend {

class PerIterationCallback {
 public:
  virtual void callback() = 0;
  virtual ~PerIterationCallback() {};
};

}  // namespace backend
}  // namespace aslam

#endif /*ASLAM_PER_ITERATION_CALLBACK_HPP*/
