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

#include "aslam/calibration/exceptions/Exception.h"

#include <sstream>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    Exception::Exception(const std::string& msg, const std::string& filename,
        size_t line, const std::string& function) :
        mMsg(msg),
        mFilename(filename),
        mFunction(function),
        mLine(line) {
      std::stringstream stream;
      if (mFunction != " ")
        stream << mFunction << ": ";
      stream << mMsg;
      if (mFilename != " ")
        stream << " [file = " << mFilename << "]";
      if (mLine)
        stream << "[line = " << mLine << "]";
      mOutputMessage = stream.str();
    }

    Exception::Exception(const Exception& other) throw() :
        mMsg(other.mMsg),
        mFilename(other.mFilename),
        mFunction(other.mFunction),
        mLine(other.mLine),
        mOutputMessage(other.mOutputMessage) {
    }

    Exception& Exception::operator = (const Exception& other) throw() {
      if (this != &other) {
        mMsg = other.mMsg;
        mFilename = other.mFilename;
        mFunction = other.mFunction;
        mLine = other.mLine;
        mOutputMessage = other.mOutputMessage;
      }
      return *this;
    }

    Exception::~Exception() throw () {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    const char* Exception::what() const throw() {
      return mOutputMessage.c_str();
    }

  }
}
