/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "aslam/calibration/core/IncrementalOptimizationProblem.h"

#include <algorithm>
#include <utility>
#include <unordered_set>
#include <iterator>

#include <aslam/backend/DesignVariable.hpp>

#include "aslam/calibration/core/OptimizationProblem.h"
#include "aslam/calibration/exceptions/OutOfBoundException.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"
#include "aslam/calibration/exceptions/NullPointerException.h"
#include "aslam/calibration/algorithms/permute.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    IncrementalOptimizationProblem::IncrementalOptimizationProblem() {
    }

    IncrementalOptimizationProblem::~IncrementalOptimizationProblem() {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    size_t IncrementalOptimizationProblem::getNumOptimizationProblems() const {
      return _optimizationProblems.size();
    }

    const OptimizationProblem*
        IncrementalOptimizationProblem::getOptimizationProblem(
        const OptimizationProblemsSPCIt& problemIt) const {
      const size_t idx =
        std::distance(_optimizationProblems.cbegin(), problemIt);
      if (idx >= _optimizationProblems.size())
        throw OutOfBoundException<size_t>(idx, _optimizationProblems.size(),
          "index out of bounds", __FILE__, __LINE__, __PRETTY_FUNCTION__);
      return _optimizationProblems.at(idx).get();
    }

    OptimizationProblem* IncrementalOptimizationProblem::getOptimizationProblem(
        const OptimizationProblemsSPIt& problemIt) {
      const size_t idx =
        std::distance(_optimizationProblems.begin(), problemIt);
      if (idx >= _optimizationProblems.size())
        throw OutOfBoundException<size_t>(idx, _optimizationProblems.size(),
          "index out of bounds", __FILE__, __LINE__, __PRETTY_FUNCTION__);
      return _optimizationProblems.at(idx).get();
    }

    const OptimizationProblem* IncrementalOptimizationProblem::
        getOptimizationProblem(size_t idx) const {
      return getOptimizationProblem(_optimizationProblems.cbegin() + idx);
    }

    OptimizationProblem* IncrementalOptimizationProblem::
        getOptimizationProblem(size_t idx) {
      return getOptimizationProblem(_optimizationProblems.begin() + idx);
    }

    const IncrementalOptimizationProblem::OptimizationProblemsSP&
        IncrementalOptimizationProblem::getOptimizationProblems() const {
      return _optimizationProblems;
    }

    bool IncrementalOptimizationProblem::
        isDesignVariableInProblem(const DesignVariable* designVariable) const {
      return _designVariablesCounts.count(designVariable);
    }

    bool IncrementalOptimizationProblem::isErrorTermInProblem(const ErrorTerm*
        errorTerm) const {
      for (auto it = _optimizationProblems.cbegin();
          it != _optimizationProblems.cend(); ++it)
        if ((*it)->isErrorTermInProblem(errorTerm))
          return true;
      return false;
    }

    const IncrementalOptimizationProblem::DesignVariablePGroups&
        IncrementalOptimizationProblem::getDesignVariablesGroups() const {
      return _designVariables;
    }

    const IncrementalOptimizationProblem::DesignVariablesP&
        IncrementalOptimizationProblem::
        getDesignVariablesGroup(size_t groupId) const {
      if (isGroupInProblem(groupId))
        return _designVariables.at(groupId);
      else
        throw OutOfBoundException<size_t>(groupId, "unknown group",
          __FILE__, __LINE__, __PRETTY_FUNCTION__);
    }

    const IncrementalOptimizationProblem::ErrorTermsSP&
        IncrementalOptimizationProblem::getErrorTerms(size_t idx) const {
      if (idx >= _optimizationProblems.size())
        throw OutOfBoundException<size_t>(idx, _optimizationProblems.size(),
          "index out of bounds", __FILE__, __LINE__, __PRETTY_FUNCTION__);
      return _optimizationProblems.at(idx)->getErrorTerms();
    }

    size_t IncrementalOptimizationProblem::getNumGroups() const {
      return _designVariables.size();
    }

    void IncrementalOptimizationProblem::
        setGroupsOrdering(const std::vector<size_t>& groupsOrdering) {
      if (groupsOrdering.size() != _groupsOrdering.size())
        throw OutOfBoundException<size_t>(groupsOrdering.size(),
          _groupsOrdering.size(), "wrong groups ordering size", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      std::unordered_set<size_t> groupsLookup;
      for (auto it = groupsOrdering.cbegin(); it != groupsOrdering.cend();
          ++it) {
        if (!isGroupInProblem(*it))
          throw OutOfBoundException<size_t>(*it, "unknown group",
            __FILE__, __LINE__, __PRETTY_FUNCTION__);
        if (groupsLookup.count(*it))
          throw OutOfBoundException<size_t>(*it, "duplicate group",
            __FILE__, __LINE__, __PRETTY_FUNCTION__);
        groupsLookup.insert(*it);
      }
      _groupsOrdering = groupsOrdering;
    }

    const std::vector<size_t>&
        IncrementalOptimizationProblem::getGroupsOrdering() const {
      return _groupsOrdering;
    }

    size_t IncrementalOptimizationProblem::
        getGroupId(const DesignVariable* designVariable) const {
      if (isDesignVariableInProblem(designVariable))
        return _designVariablesCounts.at(designVariable).second;
      else
        throw InvalidOperationException("design variable is not in the problem",
           __FILE__, __LINE__, __PRETTY_FUNCTION__);
    }

    size_t IncrementalOptimizationProblem::getGroupDim(size_t groupId) const {
      const DesignVariablesP& designVariables =
        getDesignVariablesGroup(groupId);
      size_t dim = 0;
      for (auto it = designVariables.cbegin(); it != designVariables.cend();
          ++it)
        if ((*it)->isActive())
          dim += (*it)->minimalDimensions();
      return dim;
    }

    bool IncrementalOptimizationProblem::
        isGroupInProblem(size_t groupId) const {
      return _designVariables.count(groupId);
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void IncrementalOptimizationProblem::add(
        const OptimizationProblemSP& problem) {
      if (!problem)
        throw NullPointerException("problem", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
      // update design variable counts, grouping, and storing
      const size_t numDV = problem->numDesignVariables();
      _designVariablesCounts.reserve(_designVariablesCounts.size() + numDV);
      for (size_t i = 0; i < numDV; ++i) {
        const DesignVariable* dv = problem->designVariable(i);
        const size_t groupId = problem->getGroupId(dv);
        if (!isGroupInProblem(groupId))
          _groupsOrdering.push_back(groupId);
        if (!isDesignVariableInProblem(dv)) {
          _designVariablesCounts.insert(std::make_pair(dv,
            std::make_pair(1, groupId)));
          _designVariables[groupId].push_back(dv);
        }
        else {
          if (getGroupId(dv) != groupId)
            throw InvalidOperationException("group mismatch", __FILE__,
              __LINE__, __PRETTY_FUNCTION__);
          _designVariablesCounts[dv].first++;
        }
      }

      // keep trace of the error terms of this problem
      const size_t numET = problem->numErrorTerms();
      for (size_t i = 0; i < numET; ++i) {
        const ErrorTerm* et = problem->errorTerm(i);
        if (isErrorTermInProblem(et))
          throw InvalidOperationException("error term already in the problem",
            __FILE__, __LINE__, __PRETTY_FUNCTION__);
      }

      // insert the problem
      _optimizationProblems.push_back(problem);
    }

    void IncrementalOptimizationProblem::remove(
        const OptimizationProblemsSPIt& problemIt) {

      // index bounds check
      const size_t idx =
        std::distance(_optimizationProblems.begin(), problemIt);
      if (idx >= _optimizationProblems.size())
        throw OutOfBoundException<size_t>(idx, _optimizationProblems.size(),
          "index out of bound", __FILE__, __LINE__, __PRETTY_FUNCTION__);

      // get the optimization problem to remove
      const OptimizationProblemSP& problem = _optimizationProblems.at(idx);

      // update design variable counts and remove the pointers if necessary
      const size_t numDV = problem->numDesignVariables();
      for (size_t i = 0; i < numDV; ++i) {
        const DesignVariable* dv = problem->designVariable(i);
        _designVariablesCounts[dv].first--;
        if (_designVariablesCounts[dv].first == 0) {
          const size_t groupId = getGroupId(dv);
          _designVariablesCounts.erase(dv);
          auto it = std::find(_designVariables[groupId].begin(),
            _designVariables[groupId].end(), dv);
          _designVariables[groupId].erase(it);
          if (_designVariables[groupId].empty()) {
            _designVariables.erase(groupId);
            auto it = std::find(_groupsOrdering.begin(), _groupsOrdering.end(),
              groupId);
            _groupsOrdering.erase(it);
          }
          _designVariablesBackup.erase(const_cast<DesignVariable*>(dv));
        }
      }

      // remove problem from the container
      // costly if not at the end of the container
      _optimizationProblems.erase(problemIt);
    }

    void IncrementalOptimizationProblem::remove(size_t idx) {
      remove(_optimizationProblems.begin() + idx);
    }

    void IncrementalOptimizationProblem::clear() {
      _optimizationProblems.clear();
      _designVariablesCounts.clear();
      _designVariables.clear();
      _groupsOrdering.clear();
    }

    size_t IncrementalOptimizationProblem::
        numDesignVariablesImplementation() const {
      return _designVariablesCounts.size();
    }

    IncrementalOptimizationProblem::DesignVariable*
        IncrementalOptimizationProblem::
        designVariableImplementation(size_t idx) {
      size_t groupId, idxGroup;
      getGroupId(idx, groupId, idxGroup);
      return const_cast<DesignVariable*>(
        _designVariables.at(groupId)[idxGroup]);
    }

    const IncrementalOptimizationProblem::DesignVariable*
        IncrementalOptimizationProblem::
        designVariableImplementation(size_t idx) const {
      size_t groupId, idxGroup;
      getGroupId(idx, groupId, idxGroup);
      return _designVariables.at(groupId)[idxGroup];
    };

    size_t IncrementalOptimizationProblem::IncrementalOptimizationProblem::
        numErrorTermsImplementation() const {
      size_t numErrorTerms = 0;
      for (auto it = _optimizationProblems.cbegin();
          it != _optimizationProblems.cend(); ++it)
        numErrorTerms += (*it)->numErrorTerms();
      return numErrorTerms;
    }

    IncrementalOptimizationProblem::ErrorTerm*
        IncrementalOptimizationProblem::errorTermImplementation(size_t idx) {
      size_t batchIdx, idxBatch;
      getErrorIdx(idx, batchIdx, idxBatch);
      return const_cast<ErrorTerm*>(
        _optimizationProblems.at(batchIdx)->errorTerm(idxBatch));
    }

    const IncrementalOptimizationProblem::ErrorTerm*
        IncrementalOptimizationProblem::
        errorTermImplementation(size_t idx) const {
      size_t batchIdx, idxBatch;
      getErrorIdx(idx, batchIdx, idxBatch);
      return _optimizationProblems.at(batchIdx)->errorTerm(idxBatch);
    }

    void IncrementalOptimizationProblem::
        getErrorsImplementation(const DesignVariable* dv,
        std::set<ErrorTerm*>& outErrorSet) {
      throw InvalidOperationException("not implemented (deprecated)", __FILE__,
        __LINE__, __PRETTY_FUNCTION__);
    }

    void IncrementalOptimizationProblem::
        permuteOptimizationProblems(const std::vector<size_t>& permutation) {
      permute(_optimizationProblems, permutation);
    }

    void IncrementalOptimizationProblem::permuteDesignVariables(
        const std::vector<size_t>& permutation, size_t groupId) {
      if (isGroupInProblem(groupId))
        permute(_designVariables.at(groupId), permutation);
      else
        throw OutOfBoundException<size_t>(groupId, "unknown group", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
    }

    void IncrementalOptimizationProblem::getGroupId(size_t idx, size_t& groupId,
        size_t& idxGroup) const {
      if (idx >= _designVariablesCounts.size())
        throw OutOfBoundException<size_t>(idx, _designVariablesCounts.size(),
          "index out of bounds", __FILE__, __LINE__, __PRETTY_FUNCTION__);
      size_t idxRunning = 0;
      size_t groupIdRunning = 0;
      for (auto it = _groupsOrdering.cbegin(); it != _groupsOrdering.cend();
          ++it) {
        const DesignVariablesP& designVariables = _designVariables.at(*it);
        const size_t groupSize = designVariables.size();
        if ((idxRunning + groupSize) > idx) {
          groupIdRunning = *it;
          break;
        }
        else
          idxRunning += groupSize;
      }
      groupId = groupIdRunning;
      idxGroup = idx - idxRunning;
    }

    void IncrementalOptimizationProblem::getErrorIdx(size_t idx,
        size_t& batchIdx, size_t& idxBatch) const {
      size_t idxRunning = 0;
      size_t batchIdxRunning = 0;
      bool found = false;
      for (auto it = _optimizationProblems.cbegin();
          it != _optimizationProblems.cend(); ++it) {
        const size_t batchSize = (*it)->numErrorTerms();
        if ((idxRunning + batchSize) > idx) {
          batchIdxRunning = std::distance(_optimizationProblems.cbegin(), it);
          found = true;
          break;
        }
        else
          idxRunning += batchSize;
      }
      if (!found)
        throw OutOfBoundException<size_t>(idx, "index out of bounds", __FILE__,
          __LINE__, __PRETTY_FUNCTION__);
      batchIdx = batchIdxRunning;
      idxBatch = idx - idxRunning;
    }

    void IncrementalOptimizationProblem::remove(const OptimizationProblemSP&
        problem) {
      auto it = getOptimizationProblem(problem);
      if (it != _optimizationProblems.end())
        remove(it);
      else
        throw InvalidOperationException("problem not found", __FILE__, __LINE__,
          __PRETTY_FUNCTION__);
    }

    IncrementalOptimizationProblem::OptimizationProblemsSPIt
        IncrementalOptimizationProblem::getOptimizationProblem(const
        OptimizationProblemSP& problem) {
      return std::find(_optimizationProblems.begin(),
        _optimizationProblems.end(), problem);
    }

    IncrementalOptimizationProblem::OptimizationProblemsSPIt
        IncrementalOptimizationProblem::getOptimizationProblemBegin() {
      return _optimizationProblems.begin();
    }

    IncrementalOptimizationProblem::OptimizationProblemsSPIt
        IncrementalOptimizationProblem::getOptimizationProblemEnd() {
      return _optimizationProblems.end();
    }

    void IncrementalOptimizationProblem::saveDesignVariables() {
      for (auto it = _designVariablesCounts.cbegin();
          it != _designVariablesCounts.cend(); ++it)
        it->first->getParameters(
          _designVariablesBackup[const_cast<DesignVariable*>(it->first)]);
    }

    void IncrementalOptimizationProblem::restoreDesignVariables() {
      for (auto it = _designVariablesBackup.cbegin();
          it != _designVariablesBackup.cend(); ++it)
        it->first->setParameters(_designVariablesBackup[it->first]);
    }

  }
}
