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

/** \file OutOfBoundException.h
    \brief This file defines the OutOfBoundException class, which represents any
           exceptions occuring when trying to access unallocated memory.
  */

#ifndef ASLAM_CALIBRATION_EXCEPTIONS_OUTOFBOUNDEXCEPTION_H
#define ASLAM_CALIBRATION_EXCEPTIONS_OUTOFBOUNDEXCEPTION_H

#include <cstddef>

#include <string>

#include "aslam/calibration/exceptions/Exception.h"

namespace aslam {
  namespace calibration {

    /** The class OutOfBoundException represents any exceptions occuring when
        trying to access unallocated memory.
        \brief Out of bounds exception
      */
    template <typename X> class OutOfBoundException :
      public Exception {
    public:
      /** \name Constructors/destructor
        @{
        */
      /// Constructs exception from argument and string
      OutOfBoundException(const X& argument, const std::string& msg, const
        std::string& filename = " ", size_t line = 0, const std::string&
        function = " ");
      /// Constructs exception from argument and string
      OutOfBoundException(const X& argument, const X& bound, const std::string&
        msg, const std::string& filename = " ", size_t line = 0, const
        std::string& function = " ");
      /// Copy constructor
      OutOfBoundException(const OutOfBoundException& other) throw();
      /// Assignment operator
      OutOfBoundException& operator = (const OutOfBoundException& other)
        throw();
      /// Destructor
      virtual ~OutOfBoundException() throw();
      /** @}
        */

    protected:

    };

  }
}

#include "aslam/calibration/exceptions/OutOfBoundException.tpp"

#endif // ASLAM_CALIBRATION_EXCEPTIONS_OUTOFBOUNDEXCEPTION_H
