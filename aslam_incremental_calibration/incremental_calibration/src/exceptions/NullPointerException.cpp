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

#include "aslam/calibration/exceptions/NullPointerException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    NullPointerException::NullPointerException(const std::string&
        name, const std::string& filename, size_t line, const std::string&
        function) : Exception(name + std::string(" is a null pointer"),
        filename, line, function) {
    }

    NullPointerException::NullPointerException(const NullPointerException&
        other) throw() :
        Exception(other) {
    }

    NullPointerException& NullPointerException::operator = (const
        NullPointerException& other) throw() {
      if (this != &other) {
        Exception::operator=(other);
      }
      return *this;
    }

    NullPointerException::~NullPointerException() throw () {
    }

  }
}
