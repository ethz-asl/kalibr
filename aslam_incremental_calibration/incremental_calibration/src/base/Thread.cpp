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

#include "aslam/calibration/base/Thread.h"

#include <cmath>
#include <unistd.h>
#include "aslam/calibration/base/Threads.h"
#include "aslam/calibration/exceptions/SystemException.h"
#include "aslam/calibration/exceptions/InvalidOperationException.h"

#ifdef __NR_gettid
static pid_t gettid (void) {
  return syscall(__NR_gettid);
}
#else
static pid_t gettid (void) {
  return -ENOSYS;
}
#endif

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    Thread::Identifier::Identifier(pthread_t posix) :
        mPosix(posix),
        mKernel(-1),
        mProcess(-1) {
    }

    Thread::Identifier::Identifier(const Identifier& other) :
        mPosix(other.mPosix),
        mKernel(other.mKernel),
        mProcess(other.mProcess) {
    }

    Thread::Identifier& Thread::Identifier::operator =
        (const Identifier& other) {
      if (this != &other) {
        mPosix = other.mPosix;
        mKernel = other.mKernel;
        mProcess = other.mProcess;
      }
      return *this;
    }

    Thread::Identifier::~Identifier() {
    }

    Thread::Thread(double cycle, size_t stackSize) :
        mState(initialized),
        mCancel(false),
        mPriority(inherit),
        mStackSize(stackSize),
        mCycle(cycle),
        mNumCycles(0) {
    }

    Thread::~Thread() {
      interrupt();
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    Thread::Identifier Thread::getIdentifier() const {
      Mutex::ScopedLock lock(mMutex);
      return mIdentifier;
    }

    Thread::State Thread::getState() const {
      Mutex::ScopedLock lock(mMutex);
      return mState;
    }

    Thread::State Thread::setState(State state) {
      Mutex::ScopedLock lock(mMutex);
      return safeSetState(state);
    }

    Thread::State Thread::safeSetState(State state) {
      State oldState = mState;
      if (mCancel && (state == running))
        mState = interrupting;
      else
        mState = state;
      return oldState;
    }

    Thread::Priority Thread::getPriority() const {
      Mutex::ScopedLock lock(mMutex);
      return mPriority;
    }

    void Thread::setPriority(Priority priority) {
      Mutex::ScopedLock lock(mMutex);
      safeSetPriority(priority);
    }

    void Thread::safeSetPriority(Priority priority) {
      mPriority = priority;
      if ((mIdentifier.mPosix != 0) && (priority != inherit)) {
        int policy;
        SchedulingParameter param;
        int ret = pthread_getschedparam(mIdentifier.mPosix, &policy, &param);
        if (ret)
          throw SystemException(ret,
            "Thread::safeSetPriority()::pthread_getschedparam()");
        if (!ret) {
          const int minPriority = sched_get_priority_min(policy);
          const int maxPriority = sched_get_priority_max(policy);
          if ((minPriority != -1) && (maxPriority != -1)) {
            param.sched_priority = minPriority +
              round((maxPriority - minPriority) / critical * priority);
            ret = pthread_setschedparam(mIdentifier.mPosix, policy, &param);
            if (ret)
              throw SystemException(ret,
                "Thread::safeSetPriority()::pthread_setschedparam()");
          }
        }
      }
    }

    size_t Thread::getStackSize() const {
      return mStackSize;
    }

    double Thread::getCycle() const {
      Mutex::ScopedLock lock(mMutex);
      return mCycle;
    }

    void Thread::setCycle(double cycle) {
      Mutex::ScopedLock lock(mMutex);
      mCycle = cycle;
    }

    size_t Thread::getNumCycles() const {
      Mutex::ScopedLock lock(mMutex);
      return mNumCycles;
    }

    const Timer& Thread::getTimer() const {
      return mTimer;
    }

    Condition& Thread::getTrigger() {
      return mTrigger;
    }

    const Condition& Thread::getTrigger() const {
      return mTrigger;
    }

/******************************************************************************/
/* Streaming operations                                                       */
/******************************************************************************/

    void Thread::Identifier::read(std::istream& stream) {
    }

    void Thread::Identifier::write(std::ostream& stream) const {
      stream << mPosix;
    }

    void Thread::Identifier::read(std::ifstream& stream) {
    }

    void Thread::Identifier::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    bool Thread::Identifier::operator==(const Identifier& identifier) const {
      return (mPosix == identifier.mPosix);
    }

    bool Thread::Identifier::operator!=(const Identifier& identifier) const {
      return (mPosix != identifier.mPosix);
    }

    bool Thread::Identifier::operator>(const Identifier& identifier) const {
      return (mPosix > identifier.mPosix);
    }

    bool Thread::Identifier::operator<(const Identifier& identifier) const {
      return (mPosix < identifier.mPosix);
    }

    Thread::Identifier::operator const void*() const {
      return (void*)mPosix;
    }

    void Thread::Identifier::reset() {
      mPosix = 0;
      mKernel = -1;
      mProcess = -1;
    }

    bool Thread::start(Priority priority, double wait) {
      Mutex::ScopedLock lock(mMutex);
      bool result = false;
      if (safeWait(wait)) {
        Attribute attr;
        int ret = pthread_attr_init(&attr);
        if (ret)
          throw SystemException(ret, "Thread::start()::pthread_attr_init()");
        ret = pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        if (ret)
          throw SystemException(ret,
            "Thread::start()::pthread_attr_setdetachstate()");
        ret = pthread_attr_setinheritsched(&attr, PTHREAD_INHERIT_SCHED);
        if (ret)
          throw SystemException(ret,
            "Thread::start()::pthread_attr_setinheritsched()");
        if (mStackSize) {
          ret = pthread_attr_setstacksize(&attr, mStackSize);
          if (ret)
            throw SystemException(ret,
              "Thread::start()::pthread_attr_setstacksize()");
        }
        mNumCycles = 0;
        State state = mState;
        safeSetState(starting);
        ret = pthread_create(&mIdentifier.mPosix, &attr, Thread::start, this);
        if (ret)
          throw SystemException(ret, "Thread::start()::pthread_create()");
        result = !ret;
        if (result) {
          mStarted.wait(mMutex);
          safeSetPriority(priority);
        }
        else
          safeSetState(state);
        ret = pthread_attr_destroy(&attr);
        if (ret)
          throw SystemException(ret, "Thread::start()::pthread_attr_destroy()");
      }
      return result;
    }

    bool Thread::interrupt(double wait) {
      Mutex::ScopedLock lock(mMutex);
      bool result = false;
      if (safeIsBusy()) {
        mCancel = true;
        if (mState == running)
          safeSetState(interrupting);
        mTrigger.signal();
        result = safeWait(wait);
      }
      return result;
    }

    void Thread::exit() {
      try {
        Threads::getInstance().getSelf().interrupt(0.0);
      }
      catch (...) {
        pthread_exit(0);
      }
    }

    bool Thread::wait(double seconds) const {
      Mutex::ScopedLock lock(mMutex);
      return safeWait(seconds);
    }

    bool Thread::safeWait(double seconds) const {
      bool result = true;
      if (seconds != 0.0) {
        Thread* self = 0;
        try {
          self = &Threads::getInstance().getSelf();
        }
        catch (...) {
          self = 0;
        }
        if (self == this)
          throw InvalidOperationException("Thread::safeWait(): bad wait");
        if (safeIsBusy())
          result = mCleaned.wait(mMutex, seconds);
      }
      return result;
    }

    bool Thread::exists() const {
      Mutex::ScopedLock lock(mMutex);
      return safeExists();
    }

    bool Thread::safeExists() const {
      return (mIdentifier.mPosix != 0);
    }

    bool Thread::isBusy() const {
      Mutex::ScopedLock lock(mMutex);
      return safeIsBusy();
    }

    bool Thread::safeIsBusy() const {
      return ((mState != initialized) &&
        (mState != interrupted) &&
        (mState != finished));
    }

    void* Thread::run() {
      initialize();
      mMutex.lock();
      while (!mCancel || !mNumCycles) {
        mMutex.unlock();
        mTimer.start();
        process();
        mMutex.lock();
        mTrigger.wait(mMutex, mTimer.getLeft(mCycle));
        mMutex.unlock();
        mTimer.stop();
        mMutex.lock();
        ++mNumCycles;
        if (mCycle < 0.0)
          break;
      }
      mMutex.unlock();
      cleanup();
      return 0;
    }

    void Thread::initialize() {
      Mutex::ScopedLock lock(mMutex);
      mIdentifier.mProcess = getpid();
      mIdentifier.mKernel = gettid();
      Threads::getInstance().registerThread(*this);
      safeSetState(running);
      mStarted.signal(Condition::broadcast);
    }

    void Thread::cleanup() {
      Mutex::ScopedLock lock(mMutex);
      if (mCancel)
        safeSetState(interrupted);
      else
        safeSetState(finished);
      mCancel = false;
      Threads::getInstance().unregisterThread(*this);
    //  const int ret = pthread_detach(mIdentifier.mPosix);
    //  if (ret)
    //    throw SystemException(ret, "Thread::cleanup()::pthread_detach()");
      mIdentifier.reset();
      mCleaned.signal(Condition::broadcast);
    }

    void* Thread::start(void* thread) {
      return ((Thread*)thread)->run();
    }

  }
}
