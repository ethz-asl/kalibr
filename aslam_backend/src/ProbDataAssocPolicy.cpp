#include <aslam/backend/ProbDataAssocPolicy.hpp>

#include <vector>

namespace aslam {
namespace backend {
ProbDataAssocPolicy::ProbDataAssocPolicy(ErrorTermGroups error_terms,
                                         double lambda) {
  error_terms_ = error_terms;
  scaling_factor_ = -lambda / 2;
}

void ProbDataAssocPolicy::callback() {
  for (ErrorTermGroup vect : *error_terms_) {
    bool init_max = false;
    double max_log_weight = 0;
    std::vector<double> log_weights;
    log_weights.reserve(vect->size());
    for (ErrorTermPtr error_term : *vect) {
      // Update log_weights
      double log_weight = scaling_factor_ * (error_term->getRawSquaredError());
      if (!init_max) {
        max_log_weight = log_weight;
        init_max = true;
      } else if (log_weight > max_log_weight) {
        max_log_weight = log_weight;
      }
      log_weights.push_back(log_weight);
    }

    double log_norm_constant = 0;
    for (double log_w : log_weights) {
      log_norm_constant += exp(log_w - max_log_weight);
    }
    log_norm_constant = log(log_norm_constant) + max_log_weight;

    for (std::size_t i = 0; i < vect->size(); i++) {
      boost::shared_ptr<FixedWeightMEstimator> m_estimator(
          (*vect)[i]->getMEstimatorPolicy<FixedWeightMEstimator>());
      assert(m_estimator);
      m_estimator->setWeight(log_weights[i] - log_norm_constant);
    }
  }
}
}  // namespace backend
}  // namespace aslam
