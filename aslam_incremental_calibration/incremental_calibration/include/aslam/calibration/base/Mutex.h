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

/** \file Mutex.h
    \brief This file defines the Mutex class, which provides mutex facilities
  */

#ifndef ASLAM_CALIBRATION_BASE_MUTEX_H
#define ASLAM_CALIBRATION_BASE_MUTEX_H

#include <pthread.h>

#include "aslam/calibration/base/Condition.h"
#include "aslam/calibration/base/Timer.h"

namespace aslam {
  namespace calibration {

    /** The class Mutex implements mutex facilities.
        \brief Mutex facilities
      */
    class Mutex :
      protected Condition {
      friend class Condition;
    public:
      /** \name Types definitions
        @{
        */
      /// Mutex for locking on scopes
      struct ScopedLock {
      public:
        /// Constructor
        ScopedLock(Mutex& mutex);
        /// Destructor
        ~ScopedLock();
      protected:
        /// Mutex
        Mutex* mMutex;
      };
      /** @}
        */

      /** \name Constructors/Destructor
        @{
        */
      /// Constructs mutex with parameter
      Mutex(bool recursive = false);
      /// Copy constructor
      Mutex(const Mutex& other) = delete;
      /// Assignment operator
      Mutex& operator = (const Mutex& other) = delete;
      /// Destructor
      virtual ~Mutex();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Access the number of locks of this mutex
      size_t getNumLocks() const;
      /** @}
        */

      /** \name Methods
        @{
        */
     /// Lock the mutex
      bool lock(double wait = Timer::eternal());
      /// Unlock the mutex
      void unlock();
      /// Try to lock the mutex without blocking the calling thread
      bool tryLock();
      /// Wait for the mutex to unlock
      bool waitUnlock(double seconds = Timer::eternal()) const;
      /// Check if mutex is recursive
      bool isRecursive() const;
      /// Check if mutex is locked
      bool isLocked() const;
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Safely lock the mutex
      virtual bool safeLock(double wait);
      /// Safely unlock the mutex
      virtual void safeUnlock();
      /// Safely wait eternally
      bool safeEternalWait(const Mutex& mutex) const;
      /// Safely wait until a time has elapsed
      bool safeWaitUntil(const Mutex& mutex, const Timestamp& time) const;
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Recursive mutex
      bool mRecursive;
      /// Number of locks for this mutex
      size_t mNumLocks;
      /// Mutex identifier
      mutable pthread_mutex_t mIdentifier;
      /// Owner thread
      pthread_t mOwner;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_BASE_MUTEX_H
