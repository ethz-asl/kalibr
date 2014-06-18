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

/** \file DiscreteFunction1v.h
    \brief This file defines the class DiscreteFunction1v, which is an interface
           to the univariate functions
  */

#include "aslam/calibration/functions/Function.h"

namespace aslam {
  namespace calibration {

    /** The class DiscreteFunction1v is an interface to the univariate discrete
        functions.
        \brief Univariate discrete function
      */
    template <typename Y, typename X> class DiscreteFunction<Y, X> :
      public virtual Function<Y, X> {
    public:
      /** \name Types
        @{
        */
      /// Domain type
      typedef X DomainType;
      /// Codomain type
      typedef Y CodomainType;
      /** @}
        */

      /** \name Constructors/Destructor
        @{
        */
      /// Destructor
      virtual ~DiscreteFunction();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Access the function's number of variables
      virtual size_t getNumVariables() const;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/functions/DiscreteFunction1v.tpp"
