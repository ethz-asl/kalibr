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

/** \file NullPointerException.h
    \brief This file defines the NullPointerException class, which
           represents null pointer exceptions.
  */

#ifndef ASLAM_CALIBRATION_EXCEPTIONS_NULLPOINTEREXCEPTION_H
#define ASLAM_CALIBRATION_EXCEPTIONS_NULLPOINTEREXCEPTION_H

#include <cstddef>

#include <string>

#include "aslam/calibration/exceptions/Exception.h"

namespace aslam {
  namespace calibration {

    /** The class NullPointerException represents null pointer exceptions.
        \brief Null pointer exception
      */
    class NullPointerException :
      public Exception {
    public:
      /** \name Constructors/Destructor
        @{
        */
      /// Constructs exception from message
      NullPointerException(const std::string& name = "", const std::string&
        filename = " ", size_t line = 0, const std::string& function = " ");
      /// Copy constructor
      NullPointerException(const NullPointerException& other)
        throw ();
      /// Assignment operator
      NullPointerException& operator = (const NullPointerException&
        other) throw();
      /// Destructor
      virtual ~NullPointerException() throw ();
      /** @}
        */

    protected:

    };

  }
}

#endif // ASLAM_CALIBRATION_EXCEPTIONS_NULLPOINTEREXCEPTION_H
