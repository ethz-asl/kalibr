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

/** \file DiscreteFunctionMv.h
    \brief This file defines the class DiscreteFunctionMv, which is an interface
           to the multivariate discrete functions
  */

#include <Eigen/Core>

#include "aslam/calibration/functions/Function.h"

namespace aslam {
  namespace calibration {

    /** The DiscreteFunctionMv class is an interface to the multivariate
        discrete functions.
        \brief Multivariate discrete function
      */
    template <typename Y, typename X, int M, int N>
    class DiscreteFunction :
      public virtual Function<Y, Eigen::Matrix<X, M, N> > {
    public:
      /// \cond
      // Template parameters assertion
      static_assert(M > 0 || M == Eigen::Dynamic, "M should be larger than 0!");
      static_assert(N > 0 || N == Eigen::Dynamic, "N should be larger than 0!");
      /// \endcond

      /** \name Constructors/Destructor
        @{
        */
      /// Destructor
      virtual ~DiscreteFunction();
      /** @}
        */

      /** \name Types
        @{
        */
      /// Domain type
      typedef X DomainType;
      /// Codomain type
      typedef Y CodomainType;
      /** @}
        */

    };

  }
}

#include "aslam/calibration/functions/DiscreteFunctionMv.tpp"
