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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file IncompleteGammaPFunction.h
    \brief This file defines the IncompleteGammaPFunction class, which
           represents the complementary normalized incomplete gamma function
  */

#ifndef ASLAM_CALIBRATION_FUNCTIONS_INCOMPLETEGAMMAPFUNCTION_H
#define ASLAM_CALIBRATION_FUNCTIONS_INCOMPLETEGAMMAPFUNCTION_H

#include "aslam/calibration/functions/ContinuousFunction.h"
#include "aslam/calibration/base/Serializable.h"

namespace aslam {
  namespace calibration {

    /** The IncompleteGammaPFunction class represents the complementary
        normalized incomplete gamma function.
        \brief Complementary normalized incomplete gamma function
      */
    class IncompleteGammaPFunction :
      public ContinuousFunction<double, double>,
      public virtual Serializable {
    public:
      /** \name Types
        @{
        */
      /// Variable type
      typedef ContinuousFunction<double, double>::Domain VariableType;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Constructs function with parameters
      IncompleteGammaPFunction(double alpha);
      /// Copy constructor
      IncompleteGammaPFunction(const IncompleteGammaPFunction& other);
      /// Assignment operator
      IncompleteGammaPFunction& operator =
        (const IncompleteGammaPFunction& other);
      /// Destructor
      virtual ~IncompleteGammaPFunction();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the alpha parameter
      double getAlpha() const;
      /// Sets the alpha parameter
      void setAlpha(double alpha);
      /// Access the function value for the given argument
      virtual double getValue(const VariableType& argument) const;
      /** @}
        */

    protected:
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
      /// Alpha parameter
      double mAlpha;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_FUNCTIONS_INCOMPLETEGAMMAPFUNCTION_H
