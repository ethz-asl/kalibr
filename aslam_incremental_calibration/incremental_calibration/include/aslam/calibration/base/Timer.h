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

/** \file Timer.h
    \brief This file defines the Timer class, which provides timer facilities
  */

#ifndef ASLAM_CALIBRATION_BASE_TIMER_H
#define ASLAM_CALIBRATION_BASE_TIMER_H

#include "aslam/calibration/base/Serializable.h"
#include "aslam/calibration/base/Timestamp.h"

namespace aslam {
  namespace calibration {

    /** The class Timer implements timer facilities.
        \brief Timer facilities
      */
    class Timer :
      public virtual Serializable {
    public:
      /** \name Constructors/Destructor
        @{
        */
      /// Construct the time with parameter
      Timer(bool start = false);
      /// Copy constructor
      Timer(const Timer& other);
      /// Assignment operator
      Timer& operator = (const Timer& other);
      /// Destructor
      virtual ~Timer();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Access the timer's measured period in seconds
      double getPeriod() const;
      /// Access the timer's measured frequency (equals 1.0 / period)
      double getFrequency() const;
      /// Access the timer's start time
      const Timestamp& getStartTime() const;
      /// Access the timer's time left for the specified period in seconds
      double getLeft(double period) const;

      /** @}
        */

      /** \name Methods
        @{
        */
      /// Start the timer
      void start(bool reset = true);
      /// Stop the time after the specified period has elapsed
      void stop(double period = 0.0);
      /// Reset the timer
      void reset();
      /// Wait for the specified period to elapse, but do not stop the timer
      void wait(double period) const;
      /// Sleep the specified period in seconds
      static void sleep(double period);
      /// Return a numeric value for an infinite period of time
      static double eternal();
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
      /// Starting time of the timer
      Timestamp mStartTime;
      /// Period of the timer
      double mPeriod;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_BASE_TIMER_H
