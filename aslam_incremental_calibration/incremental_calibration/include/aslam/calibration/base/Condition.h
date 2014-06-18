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

/** \file Condition.h
    \brief This file defines the Condition class, which provides condition
           facilities
  */

#ifndef ASLAM_CALIBRATION_BASE_CONDITION_H
#define ASLAM_CALIBRATION_BASE_CONDITION_H

#include <pthread.h>

#include "aslam/calibration/base/Timer.h"

namespace aslam {
  namespace calibration {

    class Mutex;

    /** The class Condition implements condition facilities.
        \brief Condition facilities
      */
    class Condition {
    public:
      /** \name Types definitions
        @{
        */
      /// Signal type
      enum SignalType {
        /// Unicast signal
        unicast,
        /// Broadcast signal
        broadcast
      };
      /** @}
        */

      /** \name Constructors/Destructor
        @{
        */
      /// Default constructor
      Condition();
      /// Copy constructor
      Condition(const Condition& other) = delete;
      /// Assignment operator
      Condition& operator = (const Condition& other) = delete;
      /// Destructor
      virtual ~Condition();
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Signal the condition
      void signal(SignalType signalType = unicast);
      /// Wait for the condition to be signaled
      bool wait(Mutex& mutex, double seconds = Timer::eternal()) const;
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Safely wait for the condition to be signaled
      bool safeWait(const Mutex& mutex, double seconds) const;
      /// Safely wait eternally for the condition to be signaled
      bool safeEternalWait(const Mutex& mutex) const;
      /// Safely wait until the specified time for the condition to be signaled
      bool safeWaitUntil(const Mutex& mutex, const Timestamp& time) const;
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Condition identifier
      mutable pthread_cond_t mIdentifier;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_BASE_CONDITION_H
