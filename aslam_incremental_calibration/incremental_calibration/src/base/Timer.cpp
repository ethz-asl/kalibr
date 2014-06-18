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

#include "aslam/calibration/base/Timer.h"

#include <errno.h>

#include <limits>

#include "aslam/calibration/base/Threads.h"
#include "aslam/calibration/exceptions/SystemException.h"

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    Timer::Timer(bool start) :
        mStartTime(0.0),
        mPeriod(0.0) {
      if (start)
        this->start();
    }

    Timer::Timer(const Timer& other) :
        mStartTime(other.mStartTime),
        mPeriod(other.mPeriod) {
    }

    Timer& Timer::operator = (const Timer& other) {
      if (this != &other) {
        mStartTime = other.mStartTime;
        mPeriod = other.mPeriod;
      }
      return *this;
    }

    Timer::~Timer() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    void Timer::read(std::istream& stream) {
    }

    void Timer::write(std::ostream& stream) const {
      stream << "starting time: " << mStartTime << std::endl
        << "period: " << mPeriod;
    }

    void Timer::read(std::ifstream& stream) {
    }

    void Timer::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double Timer::getPeriod() const {
      return mPeriod;
    }

    double Timer::getFrequency() const {
      return 1.0 / mPeriod;
    }

    const Timestamp& Timer::getStartTime() const {
      return mStartTime;
    }

    double Timer::getLeft(double period) const {
      return mStartTime + period - Timestamp::now();
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    void Timer::start(bool reset) {
      if (reset)
        this->reset();
      mStartTime = Timestamp::now();
    }

    void Timer::stop(double period) {
      sleep(getLeft(period));
      mPeriod += Timestamp::now() - mStartTime;
    }

    void Timer::reset() {
      mStartTime = 0.0;
      mPeriod = 0.0;
    }

    void Timer::wait(double period) const {
      sleep(getLeft(period));
    }

    void Timer::sleep(double period) {
      if (period > 0.0) {
        timespec time = Timestamp(period);
        Thread* self = 0;
        try {
          self = &Threads::getInstance().getSelf();
        }
        catch (...) {
          self = 0;
        }
        Thread::State threadState;
        if (self)
          threadState = self->setState(Thread::sleeping);
        if (nanosleep(&time, 0))
          throw SystemException(errno, "Timer::sleep()::nanosleep()");
        if (self)
          self->setState(threadState);
      }
    }

    double Timer::eternal() {
      return std::numeric_limits<double>::max();
    }

  }
}
