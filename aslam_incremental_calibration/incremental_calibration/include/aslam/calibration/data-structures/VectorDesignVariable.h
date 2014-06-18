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

/** \file VectorDesignVariable.h
    \brief This file defines the VectorDesignVariable class, which implements
           a vector-valued design variable.
  */

#ifndef ASLAM_CALIBRATION_DATA_VECTOR_DESIGN_VARIABLE_H
#define ASLAM_CALIBRATION_DATA_VECTOR_DESIGN_VARIABLE_H

#include <Eigen/Core>

#include <aslam/backend/DesignVariable.hpp>

#include "aslam/calibration/base/Serializable.h"

namespace aslam {
  namespace calibration {

    /** The class VectorDesignVariable implements a vector-valued design
        variable.
        \brief Vector-valued design variable
      */
    template <int M>
    class VectorDesignVariable :
      public aslam::backend::DesignVariable,
      public virtual Serializable {
    public:
      /// \cond
      // Required by Eigen for fixed-size matrices members
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      // Template parameters assertion
      static_assert(M > 0 || M == Eigen::Dynamic, "M should be larger than 0!");
      /// \endcond

      /** \name Types definitions
        @{
        */
      /// Variable container type
      typedef Eigen::Matrix<double, M, 1> Container;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructor
      VectorDesignVariable(const Container& initValue = Container::Zero());
      /// Copy constructor
      VectorDesignVariable(const VectorDesignVariable& other);
      /// Assignment operator
      VectorDesignVariable& operator = (const VectorDesignVariable& other);
      /// Destructor
      virtual ~VectorDesignVariable();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Access the value of the design variable
      const Container& getValue() const;
      /// Set the value of the design variable
      void setValue(const Container& value);
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// What is the number of dimensions of the perturbation variable.
      virtual int minimalDimensionsImplementation() const;
      /// Update the design variable.
      virtual void updateImplementation(const double* dp, int size);
      /// Revert the last state update.
      virtual void revertUpdateImplementation();
      /// Returns the content of the design variable
      virtual void getParametersImplementation(Eigen::MatrixXd& value) const;
      /// Sets the content of the design variable
      virtual void setParametersImplementation(const Eigen::MatrixXd& value);
      /** @}
        */

      /** \name Stream methods
        @{
        */
      /// Reads from standard input
      virtual void read(std::istream& stream);
      /// Writes to standard output
      virtual void write(std::ostream& stream) const;
      /// Reads from a file
      virtual void read(std::ifstream& stream);
      /// Writes to a file
      virtual void write(std::ofstream& stream) const;
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Variable container
      Container _value;
      /// Old variable container
      Container _oldValue;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/data-structures/VectorDesignVariable.tpp"

#endif // ASLAM_CALIBRATION_DATA_VECTOR_DESIGN_VARIABLE_H
