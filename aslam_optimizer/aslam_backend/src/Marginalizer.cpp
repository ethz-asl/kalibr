/*
 * Marginalizer.cpp
 *
 *  Created on: May 24, 2013
 *      Author: mbuerki
 */

#include "aslam/backend/Marginalizer.hpp"

#include <aslam/backend/DenseQrLinearSystemSolver.hpp>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <aslam/backend/DenseMatrix.hpp>

#include <iostream>

#include <sm/logging.hpp>
#include <sm/timing/Timer.hpp>

namespace aslam {
namespace backend {

void marginalize(
			std::vector<aslam::backend::DesignVariable*>& inDesignVariables,
			std::vector<aslam::backend::ErrorTerm*>& inErrorTerms,
			int numberOfInputDesignVariablesToRemove,
			bool useMEstimator,
			boost::shared_ptr<aslam::backend::MarginalizationPriorErrorTerm>& outPriorErrorTermPtr,
			Eigen::MatrixXd& outCov,
			std::vector<aslam::backend::DesignVariable*>& outDesignVariablesInRTop,
			size_t numTopRowsInCov,
			size_t /* numThreads */)
{
		  SM_WARN_STREAM_COND(inDesignVariables.size() == 0, "Zero input design variables in the marginalizer!");

		  // check for duplicates!
		  std::unordered_set<aslam::backend::DesignVariable*> inDvSetHT;
		  for(auto it = inDesignVariables.begin(); it != inDesignVariables.end(); ++it)
		  {
			  auto ret = inDvSetHT.insert(*it);
			  SM_ASSERT_TRUE(aslam::Exception, ret.second, "Error! Duplicate design variables in input list!");
		  }
		  std::unordered_set<aslam::backend::ErrorTerm*> inEtSetHT;
		  for(auto it = inErrorTerms.begin(); it != inErrorTerms.end(); ++it)
		  {
			  auto ret = inEtSetHT.insert(*it);
			  SM_ASSERT_TRUE(aslam::Exception, ret.second, "Error! Duplicate error term in input list!");
		  }
		  SM_DEBUG_STREAM("NO duplicates in input design variables or input error terms found.");

          // Partition the design varibles into removed/remaining.
		  int dimOfDesignVariablesToRemove = 0;
		  std::vector<aslam::backend::DesignVariable*> remainingDesignVariables;
		  int k = 0;
		  size_t dimOfDvsInTopBlock = 0;
		  for(std::vector<aslam::backend::DesignVariable*>::const_iterator it = inDesignVariables.begin(); it != inDesignVariables.end(); ++it)
		  {
			  if (k < numberOfInputDesignVariablesToRemove)
			  {
				  dimOfDesignVariablesToRemove += (*it)->minimalDimensions();
			  } else
			  {
				  remainingDesignVariables.push_back(*it);
			  }

			  if(dimOfDvsInTopBlock < numTopRowsInCov)
			  {
				  outDesignVariablesInRTop.push_back(*it);
			  }
			  dimOfDvsInTopBlock += (*it)->minimalDimensions();
			  k++;
		  }

		  // store original block indices to prevent side effects
		  std::vector<int> originalBlockIndices;
		  std::vector<int> originalColumnBase;
			// assign block indices
			int columnBase = 0;
			for (size_t i = 0; i < inDesignVariables.size(); ++i) {
				originalBlockIndices.push_back(inDesignVariables[i]->blockIndex());
				originalColumnBase.push_back(inDesignVariables[i]->columnBase());

				inDesignVariables[i]->setBlockIndex(i);
				inDesignVariables[i]->setColumnBase(columnBase);
			  columnBase += inDesignVariables[i]->minimalDimensions();
			}

			int dim = 0;
			std::vector<size_t> originalRowBase;
			for(std::vector<aslam::backend::ErrorTerm*>::iterator it = inErrorTerms.begin(); it != inErrorTerms.end(); ++it)
			{
				originalRowBase.push_back((*it)->rowBase());
				(*it)->setRowBase(dim);
				dim += (*it)->dimension();
			}

		  aslam::backend::DenseQrLinearSystemSolver qrSolver;
          qrSolver.initMatrixStructure(inDesignVariables, inErrorTerms, false);

		  SM_INFO_STREAM("Marginalization optimization problem initialized with " << inDesignVariables.size() << " design variables and " << inErrorTerms.size() << " error terrms");
		  SM_INFO_STREAM("The Jacobian matrix is " << dim << " x " << columnBase);

		  qrSolver.evaluateError(1, useMEstimator);
		  qrSolver.buildSystem(1, useMEstimator);


		  const Eigen::MatrixXd& jacobian = qrSolver.getJacobian();
		  const Eigen::VectorXd& b = qrSolver.e();

		  // check dimension of jacobian
		  int jrows = jacobian.rows();
		  int jcols = jacobian.cols();

		  int dimOfRemainingDesignVariables = jcols - dimOfDesignVariablesToRemove;
		  //int dimOfPriorErrorTerm = jrows;

		  // check the rank
		  Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(jacobian);
		  //lu_decomp.setThreshold(1e-20);
		  double threshold = lu_decomp.threshold();
		  int rank = lu_decomp.rank();
		  int fullRank = std::min(jacobian.rows(), jacobian.cols());
		  SM_DEBUG_STREAM("Rank of jacobian: " << rank << " (full rank: " << fullRank << ", threshold: " << threshold << ")");
		  bool rankDeficient = rank < fullRank;
		  if(rankDeficient)
		  {
			  SM_WARN("Marginalization jacobian is rank deficient!");
		  }
		  //SM_ASSERT_FALSE(aslam::Exception, rankDeficient, "Right now, we don't want the jacobian to be rank deficient - ever...");

		  Eigen::MatrixXd R_reduced;
		  Eigen::VectorXd d_reduced;
		  if (jrows < jcols)
		  {
			  SM_THROW(aslam::Exception, "underdetermined LSE!");
			  // underdetermined LSE, don't do QR
			  R_reduced = jacobian.block(0, dimOfDesignVariablesToRemove, jrows, jcols - dimOfDesignVariablesToRemove);
			  d_reduced = b;

		  } else {
			  // PTF: Do we know what will happen when the jacobian matrix is rank deficient?
			  // MB: yes, bad things!

              // do QR decomposition
			  sm::timing::Timer myTimer("QR Decomposition");
              Eigen::HouseholderQR<Eigen::MatrixXd> qr(jacobian);
			  Eigen::MatrixXd Q = qr.householderQ();
			  Eigen::MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();
			  Eigen::VectorXd d = Q.transpose()*b;
			  myTimer.stop();

			  if(numTopRowsInCov > 0)
			  {
				sm::timing::Timer myTimer("Covariance computation");
				Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(R);
				Eigen::MatrixXd Rinv = lu_decomp.inverse();
				Eigen::MatrixXd covariance = Rinv * Rinv.transpose();
				outCov = covariance.block(0, 0, numTopRowsInCov, numTopRowsInCov);
				myTimer.stop();
			  }

//			  size_t numRowsToKeep = rank - dimOfDesignVariablesToRemove;
//			  SM_ASSERT_TRUE_DBG(aslam::Exception, rankDeficient || (numRowsToKeep == dimOfRemainingDesignVariables), "must be the same if full rank!");

			  // get the top left block
			  SM_ASSERT_GE(aslam::Exception, R.rows(), numTopRowsInCov, "Cannot extract " << numTopRowsInCov << " rows of R because it only has " << R.rows() << " rows.");
			  SM_ASSERT_GE(aslam::Exception, R.cols(), numTopRowsInCov, "Cannot extract " << numTopRowsInCov << " cols of R because it only has " << R.cols() << " cols.");
			  //outRtop = R.block(0, 0, numTopRowsInRtop, numTopRowsInRtop);

              // cut off the zero rows at the bottom
              R_reduced = R.block(dimOfDesignVariablesToRemove, dimOfDesignVariablesToRemove, dimOfRemainingDesignVariables, dimOfRemainingDesignVariables);
			  //R_reduced = R.block(dimOfDesignVariablesToRemove, dimOfDesignVariablesToRemove, numRowsToKeep, dimOfRemainingDesignVariables);

              d_reduced = d.segment(dimOfDesignVariablesToRemove, dimOfRemainingDesignVariables);
              //d_reduced = d.segment(dimOfDesignVariablesToRemove, numRowsToKeep);
              //dimOfPriorErrorTerm = dimOfRemainingDesignVariables;
		  }

		  // now create the new error term
		  boost::shared_ptr<aslam::backend::MarginalizationPriorErrorTerm> err(new aslam::backend::MarginalizationPriorErrorTerm(remainingDesignVariables, d_reduced, R_reduced));

		  outPriorErrorTermPtr.swap(err);

		  // restore initial block indices to prevent side effects
          for (size_t i = 0; i < inDesignVariables.size(); ++i) {
              inDesignVariables[i]->setBlockIndex(originalBlockIndices[i]);
              inDesignVariables[i]->setColumnBase(originalColumnBase[i]);
          }
          int index = 0;
          for(std::vector<aslam::backend::ErrorTerm*>::iterator it = inErrorTerms.begin(); it != inErrorTerms.end(); ++it)
          {
              (*it)->setRowBase(originalRowBase[index++]);
          }
}


} /* namespace backend */
} /* namespace aslam */
