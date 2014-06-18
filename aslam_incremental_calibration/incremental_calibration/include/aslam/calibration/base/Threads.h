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

/** \file Threads.h
    \brief This file defines the Threads class, which provides threads
           manager facilities
  */

#ifndef ASLAM_CALIBRATION_BASE_THREADS_H
#define ASLAM_CALIBRATION_BASE_THREADS_H

#include <map>

#include "aslam/calibration/base/Singleton.h"
#include "aslam/calibration/base/Thread.h"

namespace aslam {
  namespace calibration {

    /** The class Threads implements threads manager facilities.
        \brief Threads manager facilities
      */
    class Threads :
      public Singleton<Threads> {
      friend class Singleton<Threads>;
      friend class Thread;
    public:
      /** \name Accessors
        @{
        */
      /// Access the number of thread objects
      size_t getNumThreads() const;
      /// Access the thread object associated with the calling thread
      Thread& getSelf() const;
      /// Access the thread object associated with the specified identifier
      Thread& get(const Thread::Identifier& identifier) const;
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Interrupt all registered thread objects
      void interrupt();
      /** @}
        */

    protected:
      /** \name Protected constructors/destructor
        @{
        */
      /// Default constructor
      Threads();
      /// Copy constructor
      Threads(const Threads& other) = delete;
      /// Assignment operator
      Threads& operator = (const Threads& other) = delete;
      /// Destructor
      virtual ~Threads();
      /** @}
        */

      /** \name Protected methods
        @{
        */
      /// Register a thread
      void registerThread(Thread& thread);
      /// Unregister a thread
      void unregisterThread(Thread& thread);
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Map between thread identifier and thread pointers
      std::map<Thread::Identifier, Thread*> mInstances;
      /// Mutex protecting the object
      mutable pthread_mutex_t mMutex;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_BASE_THREADS_H
