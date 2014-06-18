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

/** \file ThreadsManagerException.h
    \brief This file defines the ThreadsManagerException class, which
           is thrown whenever an exception occured in threads manager
  */

#ifndef ASLAM_CALIBRATION_EXCEPTIONS_THREADSMANAGEREXCEPTION_H
#define ASLAM_CALIBRATION_EXCEPTIONS_THREADSMANAGEREXCEPTION_H

#include <cstddef>

#include <string>

#include "aslam/calibration/exceptions/Exception.h"

namespace aslam {
  namespace calibration {

    /** The class ThreadsManagerException represents any exceptions occuring
        with threads management
        \brief Threads manager exception
      */
    template <typename X> class ThreadsManagerException :
      public Exception {
    public:
      /** \name Constructors/destructor
        @{
        */
      /// Constructs exception from argument and string
      ThreadsManagerException(const X& argument, const std::string& msg, const
        std::string& filename = " ", size_t line = 0, const std::string&
        function = " ");
      /// Copy constructor
      ThreadsManagerException(const ThreadsManagerException& other) throw ();
      /// Assignment operator
      ThreadsManagerException& operator = (const ThreadsManagerException& other)
        throw ();
      /// Destructor
      virtual ~ThreadsManagerException() throw ();
      /** @}
        */

    protected:

    };

  }
}

#include "aslam/calibration/exceptions/ThreadsManagerException.tpp"

#endif // ASLAM_CALIBRATION_EXCEPTIONS_THREADSMANAGEREXCEPTION_H
