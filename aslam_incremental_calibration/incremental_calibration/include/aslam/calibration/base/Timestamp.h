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

/** \file Timestamp.h
    \brief This file defines the Timestamp class, which provides timestamping
           facilities
  */

#ifndef ASLAM_CALIBRATION_BASE_TIMESTAMP_H
#define ASLAM_CALIBRATION_BASE_TIMESTAMP_H

#include <time.h>

#include "aslam/calibration/base/Serializable.h"

namespace aslam {
  namespace calibration {

    /** The class Timestamp implements timestamping facilities.
        \brief Timestamping facilities
      */
    class Timestamp :
      public virtual Serializable {
    public:
      /** \name Constructors/Destructor
        @{
        */
      /// Constructs timestamp object from parameter
      Timestamp(double seconds = now());
      /// Copy constructor
      Timestamp(const Timestamp& other);
      /// Assignment operator
      Timestamp& operator = (const Timestamp& other);
      /// Destructor
      virtual ~Timestamp();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Access the timestamp's value in seconds
      double getSeconds() const;
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Return the time in seconds from the timestamp
      operator double() const;
      /// Return the timespec object from the timestamp
      operator timespec() const;
      /// Equal comparison
      bool operator == (const Timestamp& timestamp) const;
      /// Equal comparison
      bool operator == (double seconds) const;
      /// Not equal comparison
      bool operator != (const Timestamp& timestamp) const;
      /// Not equal comparison
      bool operator != (double seconds) const;
      /// Bigger comparison
      bool operator > (const Timestamp& timestamp) const;
      /// Bigger comparison
      bool operator > (double seconds) const;
      /// Smaller comparison
      bool operator < (const Timestamp& timestamp) const;
      /// Smaller comparison
      bool operator < (double seconds) const;
      /// Bigger or equal comparison
      bool operator >= (const Timestamp& timestamp) const;
      /// Bigger or equal comparison
      bool operator >= (double seconds) const;
      /// Smaller or equal comparison
      bool operator <= (const Timestamp& timestamp) const;
      /// Smaller or equal comparison
      bool operator <= (double seconds) const;
      /// Add 2 timestamps
      Timestamp& operator += (double seconds);
      /// Substract 2 timestamps
      Timestamp& operator -= (double seconds);
      /// Add seconds to timestamp
      double operator + (double seconds) const;
      /// Add another timestamp
      double operator + (const Timestamp& timestamp) const;
      /// Substract another timestamp
      double operator - (const Timestamp& timestamp) const;
      /// Substract seconds
      double operator - (double seconds) const;
      /// Returns the system time in s
      static double now();
      /// Returns the date of the system in string
      static std::string getDate();
      /// Returns the date from timestamp in seconds
      static std::string getDate(double seconds);
      /** @}
        */

    protected:
      /** \name Stream methods
        @{
        */
      /// Reads from standard input
      virtual void read(std::istream& stream);
      /// Writes to standard output
      virtual void write(std::ostream& stream) const;
      /// Reads from a file
      virtual void read(std::ifstream& stream);
      /// Writes to a file
      virtual void write(std::ofstream& stream) const;
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Seconds in the timestamp
      double mSeconds;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_BASE_TIMESTAMP_H
