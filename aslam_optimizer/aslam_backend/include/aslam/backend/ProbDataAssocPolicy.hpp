#ifndef ASLAM_PROB_DATA_ASSOC_POLICY_HPP
#define ASLAM_PROB_DATA_ASSOC_POLICY_HPP

#include <math.h>

#include <boost/shared_ptr.hpp>

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/PerIterationCallback.hpp>

#include <vector>

namespace aslam {
namespace backend {

// Update the weights of the error terms using a gaussian of the squared error,
// i.e: exp(-lambda/2*(y -f(x))^2)
// The matrix of error terms contains, in each row, the error terms whose
// weights must be normalized together
class ProbDataAssocPolicy : public PerIterationCallback {
 public:
  typedef boost::shared_ptr<ErrorTerm> ErrorTermPtr;
  typedef boost::shared_ptr<std::vector<ErrorTermPtr>> ErrorTermGroup;
  typedef boost::shared_ptr<std::vector<ErrorTermGroup>> ErrorTermGroups;

  ProbDataAssocPolicy(ErrorTermGroups error_terms, double lambda);
  // The optimizer will call this function before each iteration.
  void callback();

 private:
  ErrorTermGroups error_terms_;
  double scaling_factor_;
};
}  // namespace backend
}  // namespace aslam

#endif /*ASLAM_PROB_DATA_ASSOC_POLICY_HPP*/
