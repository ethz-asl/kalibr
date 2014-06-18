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

/** \file BadArgumentException.h
    \brief This file defines the BadArgumentException class, which
           is thrown whenever the arguments of a function are invalid
  */

#ifndef ASLAM_CALIBRATION_EXCEPTIONS_BADARGUMENTEXCEPTION_H
#define ASLAM_CALIBRATION_EXCEPTIONS_BADARGUMENTEXCEPTION_H

#include <cstddef>

#include <string>

#include "aslam/calibration/exceptions/Exception.h"

namespace aslam {
  namespace calibration {

    /** The class BadArgumentException represents any
        exceptions occuring when the arguments passed to a function are invalid.
        \brief Bad argument exception
      */
    template <typename X> class BadArgumentException :
      public Exception {
    public:
      /** \name Constructors/destructor
        @{
        */
      /// Constructs exception from argument and string
      BadArgumentException(const X& argument, const std::string& msg, const
        std::string& filename = " ", size_t line = 0, const std::string&
        function = " ");
      /// Copy constructor
      BadArgumentException(const BadArgumentException& other) throw();
      /// Assignment operator
      BadArgumentException& operator = (const BadArgumentException& other)
        throw();
      /// Destructor
      virtual ~BadArgumentException() throw();
      /** @}
        */

    protected:

    };

  }
}

#include "aslam/calibration/exceptions/BadArgumentException.tpp"

#endif // ASLAM_CALIBRATION_EXCEPTIONS_BADARGUMENTEXCEPTION_H
