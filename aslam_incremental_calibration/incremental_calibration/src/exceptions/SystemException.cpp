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

#include "aslam/calibration/exceptions/SystemException.h"

#include <cstring>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    SystemException::SystemException(int errNo, const
        std::string& msg, const std::string& filename, size_t line, const
        std::string& function) :
        Exception(msg + std::string(strerror(errNo)), filename, line,
        function) {
    }

    SystemException::SystemException(const SystemException& other) throw() :
        Exception(other) {
    }

    SystemException& SystemException::operator =
        (const SystemException& other) throw() {
      if (this != &other) {
        Exception::operator=(other);
      }
      return *this;
    }

    SystemException::~SystemException() throw() {
    }

  }
}
