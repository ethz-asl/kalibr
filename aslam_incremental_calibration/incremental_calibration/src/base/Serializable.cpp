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

#include "aslam/calibration/base/Serializable.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors/Destructor                                                    */
/******************************************************************************/

    Serializable::~Serializable() {
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    std::ostream& operator << (std::ostream& stream, const Serializable& obj) {
      obj.write(stream);
      return stream;
    }

    std::istream& operator >> (std::istream& stream, Serializable& obj) {
      obj.read(stream);
      return stream;
    }

    std::ofstream& operator << (std::ofstream& stream,
        const Serializable& obj) {
      obj.write(stream);
      return stream;
    }

    std::ifstream& operator >> (std::ifstream& stream, Serializable& obj) {
      obj.read(stream);
      return stream;
    }

  }
}
