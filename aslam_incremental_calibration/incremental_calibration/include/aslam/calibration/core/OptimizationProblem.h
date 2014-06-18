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

/** \file OptimizationProblem.h
    \brief This file defines the OptimizationProblem class, which is
           a container for an optimization problem.
  */

#ifndef ASLAM_CALIBRATION_CORE_OPTIMIZATION_PROBLEM_H
#define ASLAM_CALIBRATION_CORE_OPTIMIZATION_PROBLEM_H

#include <set>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

#include <aslam/backend/OptimizationProblemBase.hpp>

namespace aslam {
  namespace backend {

    class DesignVariable;
    class ErrorTerm;

  }
  namespace calibration {

    /** The class OptimizationProblem implements a container for an optimization
        problem.
        \brief Optimization problem
      */
    class OptimizationProblem :
      public aslam::backend::OptimizationProblemBase {
    public:
      /** \name Types definitions
        @{
        */
      /// Design variable type
      typedef aslam::backend::DesignVariable DesignVariable;
      /// Design variable type (shared pointer)
      typedef boost::shared_ptr<DesignVariable> DesignVariableSP;
      /// Fast lookup container for design variables pointers
      typedef std::unordered_map<const DesignVariable*, size_t>
        DesignVariablesP;
      /// Error term type
      typedef aslam::backend::ErrorTerm ErrorTerm;
      /// Error term type (shared pointer)
      typedef boost::shared_ptr<ErrorTerm> ErrorTermSP;
      /// Fast lookup container for error terms pointers
      typedef std::unordered_set<const ErrorTerm*> ErrorTermsP;
      /// Container for design variables (shared pointer)
      typedef std::vector<DesignVariableSP> DesignVariablesSP;
      /// Container for error terms (shared pointer)
      typedef std::vector<ErrorTermSP> ErrorTermsSP;
      /// Container for design variable groups
      typedef std::unordered_map<size_t, DesignVariablesSP>
        DesignVariableSPGroups;
      /// Container for design variables saving/restoring
      typedef std::unordered_map<DesignVariable*, Eigen::MatrixXd>
        DesignVariablesBackup;
      /// Self type
      typedef OptimizationProblem Self;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      OptimizationProblem();
      /// Copy constructor
      OptimizationProblem(const Self& other) = delete;
      /// Copy assignment operator
      OptimizationProblem& operator = (const Self& other) = delete;
      /// Move constructor
      OptimizationProblem(Self&& other) = delete;
      /// Move assignment operator
      OptimizationProblem& operator = (Self&& other) = delete;
      /// Destructor
      virtual ~OptimizationProblem();
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Inserts a design variable into the problem
      void addDesignVariable(const DesignVariableSP& designVariable,
        size_t groupId = 0);
      /// Checks if a design variable is in the problem
      bool isDesignVariableInProblem(const DesignVariable* designVariable)
        const;
      /// Inserts an error term into the problem
      void addErrorTerm(const ErrorTermSP& errorTerm);
      /// Checks if an error term is in the problem
      bool isErrorTermInProblem(const ErrorTerm* errorTerm) const;
      /// Permutes the error terms
      void permuteErrorTerms(const std::vector<size_t>& permutation);
      /// Permutes the design variables in a group
      void permuteDesignVariables(const std::vector<size_t>& permutation,
        size_t groupId);
      /// Saves the state of the design variables
      void saveDesignVariables();
      /// Restores the state of the design variables
      void restoreDesignVariables();
      /// Clears the optimization problem
      void clear();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the design variables groups
      const DesignVariableSPGroups& getDesignVariablesGroups() const;
      /// Returns the design variables associated with a group
      const DesignVariablesSP& getDesignVariablesGroup(size_t groupId) const;
      /// Returns the error terms
      const ErrorTermsSP& getErrorTerms() const;
      /// Returns the number of groups
      size_t getNumGroups() const;
      /// Sets the groups ordering
      void setGroupsOrdering(const std::vector<size_t>& groupsOrdering);
      /// Returns the groups ordering
      const std::vector<size_t>& getGroupsOrdering() const;
      /// Returns the group id of a design variable
      size_t getGroupId(const DesignVariable* designVariable) const;
      /// Returns the dimension of a group
      size_t getGroupDim(size_t groupId) const;
      /// Checks if a group is in the problem
      bool isGroupInProblem(size_t groupId) const;
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Returns the number of design variables in the problem
      virtual size_t numDesignVariablesImplementation() const;
      /// Returns design variable indexed by idx
      virtual DesignVariable* designVariableImplementation(size_t idx);
      /// Returns design variable indexed by idx
      virtual const DesignVariable* designVariableImplementation(size_t idx)
        const;
      /// Returns the number of error terms in the problem
      virtual size_t numErrorTermsImplementation() const;
      /// Returns error term indexed by idx
      virtual ErrorTerm* errorTermImplementation(size_t idx);
      /// Returns error term indexed by idx
      virtual const ErrorTerm* errorTermImplementation(size_t idx) const;
      /// Returns error terms associated with a design variable
      virtual void getErrorsImplementation(const DesignVariable* dv,
        std::set<ErrorTerm*>& outErrorSet);
      /// Returns the group id an index falls in
      void getGroupId(size_t idx, size_t& groupId, size_t& idxGroup) const;
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Lookup structure for design variables pointers
      DesignVariablesP _designVariablesLookup;
      /// Lookup structure for error terms pointers
      ErrorTermsP _errorTermsLookup;
      /// Storage for the design variables shared pointers in groups
      DesignVariableSPGroups _designVariables;
      /// Storage for the error terms shared pointers
      ErrorTermsSP _errorTerms;
      /// Groups ordering
      std::vector<size_t> _groupsOrdering;
      /// Backup for design variables
      DesignVariablesBackup _designVariablesBackup;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_CORE_OPTIMIZATION_PROBLEM_H
