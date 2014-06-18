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

#include "aslam/calibration/base/Threads.h"
#include "aslam/calibration/exceptions/SystemException.h"
#include "aslam/calibration/exceptions/ThreadsManagerException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    Threads::Threads() {
      pthread_mutex_init(&mMutex, 0);
    }

    Threads::~Threads() {
      interrupt();
      const int ret = pthread_mutex_destroy(&mMutex);
      if (ret)
        throw SystemException(ret,
          "Threads::~Threads()::pthread_mutex_destroy()");
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    size_t Threads::getNumThreads() const {
      int ret = pthread_mutex_lock(&mMutex);
      if (ret)
        throw SystemException(ret,
          "Threads::getNumThreads()::pthread_mutex_lock()");
      const size_t numThreads = mInstances.size();
      pthread_mutex_unlock(&mMutex);
      if (ret)
        throw SystemException(ret,
          "Threads::getNumThreads()::pthread_mutex_unlock()");
      return numThreads;
    }

    Thread& Threads::getSelf() const {
      return get(pthread_self());
    }

    Thread& Threads::get(const Thread::Identifier& identifier) const {
      Thread* thread = 0;
      int ret = pthread_mutex_lock(&mMutex);
      if (ret)
        throw SystemException(ret, "Threads::get()::pthread_mutex_lock()");
      auto it = mInstances.find(identifier);
      if (it != mInstances.end())
        thread = it->second;
      ret = pthread_mutex_unlock(&mMutex);
      if (ret)
        throw SystemException(ret, "Threads::get()::pthread_mutex_unlock()");
      if (thread != 0)
        return *thread;
      else
        throw ThreadsManagerException<Thread::Identifier>(identifier,
          "Threads::get(): thread not registered");
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void Threads::interrupt() {
      int ret = pthread_mutex_lock(&mMutex);
      if (ret)
        throw SystemException(ret,
          "Threads::interrupt()::pthread_mutex_lock()");
      while (!mInstances.empty()) {
        Thread* thread = mInstances.begin()->second;
        ret = pthread_mutex_unlock(&mMutex);
        if (ret)
          throw SystemException(ret,
            "Threads::interrupt()::pthread_mutex_unlock()");
        thread->interrupt();
        ret = pthread_mutex_lock(&mMutex);
        if (ret)
          throw SystemException(ret,
            "Threads::interrupt()::pthread_mutex_lock()");
      }
      ret = pthread_mutex_unlock(&mMutex);
        if (ret)
          throw SystemException(ret,
            "Threads::interrupt()::pthread_mutex_unlock()");
    }

    void Threads::registerThread(Thread& thread) {
      int ret = pthread_mutex_lock(&mMutex);
      if (ret)
        throw SystemException(ret,
          "Threads::registerThread()::pthread_mutex_lock()");
      if (mInstances.find(thread.mIdentifier) == mInstances.end())
        mInstances[thread.mIdentifier] = &thread;
      else {
        ret = pthread_mutex_unlock(&mMutex);
        if (ret)
          throw SystemException(ret,
            "Threads::registerThread()::pthread_mutex_unlock()");
        throw ThreadsManagerException<Thread::Identifier>(thread.mIdentifier,
          "Threads::registerThread(): thread already registered");
      }
      ret = pthread_mutex_unlock(&mMutex);
      if (ret)
        throw SystemException(ret,
          "Threads::registerThread()::pthread_mutex_unlock()");
    }

    void Threads::unregisterThread(Thread& thread) {
      int ret = pthread_mutex_lock(&mMutex);
      if (ret)
        throw SystemException(ret,
          "Threads::unregisterThread()::pthread_mutex_lock()");
      if (mInstances.find(thread.mIdentifier) != mInstances.end())
        mInstances.erase(thread.mIdentifier);
      else {
        ret = pthread_mutex_unlock(&mMutex);
        if (ret)
          throw SystemException(ret,
            "Threads::unregisterThread()::pthread_mutex_unlock()");
        throw ThreadsManagerException<Thread::Identifier>(thread.mIdentifier,
          "Threads::unregisterThread(): thread not registered");
      }
      ret = pthread_mutex_unlock(&mMutex);
      if (ret)
        throw SystemException(ret,
          "Threads::unregisterThread()::pthread_mutex_unlock()");
    }

 }
}
