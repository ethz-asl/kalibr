/*
 * Marginalizer.h
 *
 *  Created on: May 24, 2013
 *      Author: mbuerki
 */

#ifndef MARGINALIZER_H_
#define MARGINALIZER_H_

#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/MarginalizationPriorErrorTerm.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <boost/shared_ptr.hpp>

namespace aslam {
namespace backend {

/// \brief Marginalizes out the given design variables
///
///	\param[IN] inDesignVariables list of input design variables to be marginalized
/// \param[IN] inErrorTerms list of all error terms related to the input design variables
/// \param[IN] numberOfInputDesignVariablesToRemove Number of input design variables to be removed.
/// 											The functions removes this many input design variables, starting at the beginning of
///												the list of input design variables. Therefore, the ordering of the design variables in the
///												input desing variables list matters!
/// \param[IN] useMEstimator					Wheter or not to use an M-Estimator in the QR sovler.
/// \param[OUT] outPriorErrorTermPtr			Shared pointer to the resulting marginalized prior error term.
///
void marginalize(
			std::vector<aslam::backend::DesignVariable*>& inDesignVariables,
			std::vector<aslam::backend::ErrorTerm*>& inErrorTerms,
			int numberOfInputDesignVariablesToRemove,
			bool useMEstimator,
			boost::shared_ptr<aslam::backend::MarginalizationPriorErrorTerm>& outPriorErrorTermPtr,
			Eigen::MatrixXd& outRtop,
			std::vector<aslam::backend::DesignVariable*>& designVariablesInvolvedInRtop,
			size_t numTopRowsInRtop = 0,
			size_t numThreads = 1
		);
} /* namespace backend */
} /* namespace aslam */
#endif /* MARGINALIZER_H_ */
