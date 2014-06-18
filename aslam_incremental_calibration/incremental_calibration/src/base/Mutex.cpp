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

#include "aslam/calibration/exceptions/SystemException.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"
#include "aslam/calibration/base/Mutex.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    Mutex::ScopedLock::ScopedLock(Mutex& mutex) :
        mMutex(&mutex) {
      mMutex->lock();
    }

    Mutex::ScopedLock::~ScopedLock() {
      mMutex->unlock();
    }

    Mutex::Mutex(bool recursive) :
        mRecursive(recursive),
        mNumLocks(0),
        mOwner(0) {
      pthread_mutex_init(&mIdentifier, 0);
    }

    Mutex::~Mutex() {
      const int ret = pthread_mutex_destroy(&mIdentifier);
      if (ret)
        throw SystemException(ret, "Mutex::~Mutex()::pthread_mutex_destroy()");
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    size_t Mutex::getNumLocks() const {
      int ret = pthread_mutex_lock(&mIdentifier);
      if (ret)
        throw SystemException(ret,
          "Mutex::getNumLocks()::pthread_mutex_lock()");
      const size_t numLocks = mNumLocks;
      ret = pthread_mutex_unlock(&mIdentifier);
      if (ret)
        throw SystemException(ret,
          "Mutex:getNumLocks()::pthread_mutex_unlock()");
      return numLocks;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    bool Mutex::lock(double wait) {
      bool result = false;
      int ret = pthread_mutex_lock(&mIdentifier);
      if (ret)
        throw SystemException(ret, "Mutex::lock()::pthread_mutex_lock()");
      try {
        result = safeLock(wait);
      }
      catch (...) {
        ret = pthread_mutex_unlock(&mIdentifier);
        if (ret)
          throw SystemException(ret, "Mutex::lock()::pthread_mutex_unlock()");
        throw;
      }
      ret = pthread_mutex_unlock(&mIdentifier);
      if (ret)
        throw SystemException(ret, "Mutex::lock()::pthread_mutex_unlock()");
      return result;
    }

    void Mutex::unlock() {
      int ret = pthread_mutex_lock(&mIdentifier);
      if (ret)
        throw SystemException(ret, "Mutex::unlock()::pthread_mutex_lock()");
      try {
        safeUnlock();
      }
      catch (...) {
        ret = pthread_mutex_unlock(&mIdentifier);
        if (ret)
          throw SystemException(ret, "Mutex::unlock()::pthread_mutex_unlock()");
        throw;
      }
      ret = pthread_mutex_unlock(&mIdentifier);
      if (ret)
        throw SystemException(ret, "Mutex::unlock()::pthread_mutex_unlock()");
    }

    bool Mutex::waitUnlock(double seconds) const {
      int ret = pthread_mutex_lock(&mIdentifier);
      if (ret)
        throw SystemException(ret, "Mutex::waitUnlock()::pthread_mutex_lock()");
      const bool result = safeWait(*this, seconds);
      if (result)
        ((Mutex*)this)->signal();
      ret = pthread_mutex_unlock(&mIdentifier);
      if (ret)
        throw SystemException(ret,
          "Mutex::waitUnlock()::pthread_mutex_unlock()");
      return result;
    }

    bool Mutex::tryLock() {
      return lock(0.0);
    }

    bool Mutex::isRecursive() const {
      return mRecursive;
    }

    bool Mutex::isLocked() const {
      int ret = pthread_mutex_lock(&mIdentifier);
      if (ret)
        throw SystemException(ret, "Mutex::isLocked()::pthread_mutex_lock()");
      const bool result = mNumLocks;
      ret = pthread_mutex_unlock(&mIdentifier);
      if (ret)
        throw SystemException(ret, "Mutex::isLocked()::pthread_mutex_unlock()");
      return result;
    }

    bool Mutex::safeLock(double wait) {
      bool result = true;
      if (pthread_self() == mOwner) {
        if (!mRecursive)
          throw InvalidOperationException("Mutex::safeLock(): deadlock");
        else
          ++mNumLocks;
      }
      else {
        if (mNumLocks)
          result = safeWait(*this, wait);
        if (result) {
          ++mNumLocks;
          mOwner = pthread_self();
        }
      }
      return result;
    }

    void Mutex::safeUnlock() {
      if (!mNumLocks)
        throw InvalidOperationException("Mutex::safeUnlock(): bad operation");
      if (pthread_self() != mOwner)
        throw InvalidOperationException("Mutex::safeUnlock(): bad permissions");
      --mNumLocks;
      if (!mNumLocks) {
        mOwner = 0;
        signal();
      }
    }

    bool Mutex::safeEternalWait(const Mutex& mutex) const {
      bool result = true;
      while (result && mNumLocks)
        result = !pthread_cond_wait(&(Condition::mIdentifier), &mIdentifier);
      return result;
    }

    bool Mutex::safeWaitUntil(const Mutex& mutex, const Timestamp& time) const {
      bool result = true;
      timespec abstime = time;
      while (result && mNumLocks)
        result = !pthread_cond_timedwait(&(Condition::mIdentifier),
          &mIdentifier, &abstime);
      return result;
    }

  }
}
