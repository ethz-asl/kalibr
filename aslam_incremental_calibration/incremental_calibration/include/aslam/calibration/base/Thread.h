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

/** \file Thread.h
    \brief This file defines the Thread class, which provides threading
           facilities
  */

#ifndef ASLAM_CALIBRATION_BASE_THREAD_H
#define ASLAM_CALIBRATION_BASE_THREAD_H

#include <pthread.h>

#include "aslam/calibration/base/Timer.h"
#include "aslam/calibration/base/Mutex.h"
#include "aslam/calibration/base/Serializable.h"

namespace aslam {
  namespace calibration {

    /** The class Thread implements threading facilities.
        \brief Threading facilities
      */
    class Thread {
      friend class Threads;
      friend class Timer;
      friend class Condition;
    public:
      /** \name Types definitions
        @{
        */
      /// Thread's attribute
      typedef pthread_attr_t Attribute;
      /// Thread's scheduling parameter
      typedef sched_param SchedulingParameter;
      /// Thread state
      enum State {
        /// Thread initialized
        initialized,
        /// Thread starting
        starting,
        /// Thread running
        running,
        /// Thread sleeping
        sleeping,
        /// Thread waiting
        waiting,
        /// Thread interrupting
        interrupting,
        /// Thread interrupted
        interrupted,
        /// Thread finished
        finished
      };
      /// Thread priority
      enum Priority {
        /// Inherits priority
        inherit,
        /// Idle priority
        idle,
        /// Lowest priority
        lowest,
        /// Low priority
        low,
        /// Normal priority
        normal,
        /// High priority
        high,
        /// Highest priority
        highest,
        /// Critical priority
        critical
      };
      /// Thread identifier
      struct Identifier :
        public Serializable {
      public:
        /// C
        /// Posix identifier
        pthread_t mPosix;
        /// Kernel
        pid_t mKernel;
        /// Process
        pid_t mProcess;
        /// Constructor with parameter
        Identifier(pthread_t posix = 0);
        /// Copy constructor
        Identifier(const Identifier& other);
        /// Assignment operator
        Identifier& operator = (const Identifier& other);
        /// Destructor
        virtual ~Identifier();
        /// Thread identifier equals comparison
        bool operator==(const Identifier& identifier) const;
        /// Thread identifier not equals comparison
        bool operator!=(const Identifier& identifier) const;
        /// Thread identifier greater comparison
        bool operator>(const Identifier& identifier) const;
        /// Thread identifier lower comparison
        bool operator<(const Identifier& identifier) const;
        /// Thread identifier conversions
        operator const void*() const;
        /// Reset the identifier
        void reset();
        /// Reads from standard input
        virtual void read(std::istream& stream);
        /// Writes to standard output
        virtual void write(std::ostream& stream) const;
        /// Reads from a file
        virtual void read(std::ifstream& stream);
        /// Writes to a file
        virtual void write(std::ofstream& stream) const;
      };

      /** @}
        */

      /** \name Constructors/Destructor
        @{
        */
      /// Constructs thread with parameter
      Thread(double cycle = 0.0, size_t stackSize = 0);
      /// Copy constructor
      Thread(const Thread& other) = delete;
      /// Assignment operator
      Thread& operator = (const Thread& other) = delete;
      /// Destructor
      virtual ~Thread();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Access the thread's identifier
      Identifier getIdentifier() const;
      /// Access the thread's state
      State getState() const;
      /// Access the thread's priority
      Priority getPriority() const;
      /// Sets the thread's priority
      void setPriority(Priority priority);
      /// Access the thread's stack size
      size_t getStackSize() const;
      /// Access the thread's cycle period in seconds
      double getCycle() const;
      /// Sets the thread's cycle period in seconds
      void setCycle(double cycle);
      /// Access the thread's number of cycles performed
      size_t getNumCycles() const;
      /// Access the thread's timer
      const Timer& getTimer() const;
      /// Access the thread's trigger
      Condition& getTrigger();
      /// Access the thread's trigger
      const Condition& getTrigger() const;
      /** @}
        */

      /** \name Methods
        @{
        */
      /// Start the thread
      bool start(Priority priority = inherit, double wait = Timer::eternal());
      /// Interrupt the thread
      bool interrupt(double wait = Timer::eternal());
      /// Exit the calling thread
      static void exit();
      /// Wait until the thread has finished execution or the specified time
      bool wait(double seconds = Timer::eternal()) const;
      /// Does the thread exist
      bool exists() const;
      /// Is the thread busy
      bool isBusy() const;
      /** @}
        */

    protected:
      /** \name Protected methods
        @{
        */
      /// Access the thread's state and returns its former state
      State setState(State state);
      /// Safely access the thread's state
      virtual State safeSetState(State state);
      /// Safely access the thread's priority
      virtual void safeSetPriority(Priority priority);
      /// Run all thread operations
      virtual void* run();
      /// Do initialization
      virtual void initialize();
      /// Do computational processing
      virtual void process() = 0;
      /// Do cleanup
      virtual void cleanup();
      /// Safely wait until the thread has finished execution or time elapsed
      virtual bool safeWait(double seconds) const;
      /// Safe thread exists
      virtual bool safeExists() const;
      /// Safe thread is busy
      virtual bool safeIsBusy() const;
      /// Start the given thread object
      static void* start(void* thread);
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Thread's identifier
      Identifier mIdentifier;
      /// Thread's state
      State mState;
      /// Cancel flag
      bool mCancel;
      /// Threads' priority
      Priority mPriority;
      /// Thread's stack size
      size_t mStackSize;
      /// Thread's cycle
      double mCycle;
      /// Thread's number of cycles
      size_t mNumCycles;
      /// Mutex protecting the object
      mutable Mutex mMutex;
      /// Start condition
      Condition mStarted;
      /// Trigger condition
      Condition mTrigger;
      /// Cleaned condition
      Condition mCleaned;
      /// Thread's timer
      Timer mTimer;
      /** @}
        */

    };

  }
}

#endif // ASLAM_CALIBRATION_BASE_THREAD_H
