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

#include "aslam/calibration/base/Condition.h"

#include "aslam/calibration/base/Mutex.h"
#include "aslam/calibration/base/Threads.h"
#include "aslam/calibration/exceptions/SystemException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    Condition::Condition() {
      pthread_cond_init(&mIdentifier, 0);
    }

    Condition::~Condition() {
      const int ret = pthread_cond_destroy(&mIdentifier);
      if (ret)
        throw SystemException(ret,
          "Condition::~Condition()::pthread_cond_destroy()");
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void Condition::signal(SignalType signalType) {
      if (signalType == broadcast)
        pthread_cond_broadcast(&mIdentifier);
      else
        pthread_cond_signal(&mIdentifier);
    }

    bool Condition::wait(Mutex& mutex, double seconds) const {
      bool result = true;
      if (seconds > 0.0) {
        int ret = pthread_mutex_lock(&mutex.mIdentifier);
        if (ret)
          throw SystemException(ret, "Condition::wait()::pthread_mutex_lock()");
        mutex.safeUnlock();
        result = safeWait(mutex, seconds);
        mutex.safeLock(Timer::eternal());
        ret = pthread_mutex_unlock(&mutex.mIdentifier);
        if (ret)
          throw SystemException(ret,
            "Condition::wait()::pthread_mutex_unlock()");
      }
      return result;
    }

    bool Condition::safeWait(const Mutex& mutex, double seconds) const {
      bool result = true;
      Thread* self = 0;
      try {
        self = &Threads::getInstance().getSelf();
      }
      catch (...) {
        self = 0;
      }
      Thread::State threadState;
      if (self) {
        if (&mutex == &self->mMutex)
          threadState = self->safeSetState(Thread::waiting);
        else 
          threadState = self->setState(Thread::waiting);
      }
      if (seconds == Timer::eternal())
        result = safeEternalWait(mutex);
      else if (seconds > 0.0)
        result = safeWaitUntil(mutex, Timestamp(Timestamp::now() + seconds));
      if (self) {
        if (&mutex == &self->mMutex)
          self->safeSetState(threadState);
        else 
          self->setState(threadState);
      }
      return result;
    }

    bool Condition::safeEternalWait(const Mutex& mutex) const {
      bool result = true;
      result = !pthread_cond_wait(&mIdentifier, &mutex.mIdentifier);
      if (result && mutex.mNumLocks)
        result = mutex.safeEternalWait(mutex);
      return result;
    }

    bool Condition::safeWaitUntil(const Mutex& mutex, const Timestamp& time) 
        const {
      bool result = true;
      timespec abstime = time;
      result = !pthread_cond_timedwait(&mIdentifier, &mutex.mIdentifier,
        &abstime);
      if (result && mutex.mNumLocks)
        result = mutex.safeWaitUntil(mutex, time);
      return result;
    }

  }
}
