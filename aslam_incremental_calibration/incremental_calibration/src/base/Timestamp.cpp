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

#include "aslam/calibration/base/Timestamp.h"

#include <sys/time.h>

namespace aslam {
  namespace calibration {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

    Timestamp::Timestamp(double seconds) :
        mSeconds(seconds) {
    }

    Timestamp::Timestamp(const Timestamp& other) :
        mSeconds(other.mSeconds) {
    }

    Timestamp& Timestamp::operator = (const Timestamp& other) {
      if (this != &other) {
        mSeconds = other.mSeconds;
      }
      return *this;
    }

    Timestamp::~Timestamp() {
    }

/******************************************************************************/
/* Stream operations                                                          */
/******************************************************************************/

    void Timestamp::read(std::istream& stream) {
    }

    void Timestamp::write(std::ostream& stream) const {
      stream << "seconds: " << mSeconds;
    }

    void Timestamp::read(std::ifstream& stream) {
    }

    void Timestamp::write(std::ofstream& stream) const {
    }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

    double Timestamp::getSeconds() const {
      return mSeconds;
    }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

    double Timestamp::now() {
      struct timeval time;
      gettimeofday(&time, 0);
      return time.tv_sec + time.tv_usec / 1e6;
    }

    std::string Timestamp::getDate() {
      struct timeval time;
      gettimeofday(&time, 0);
      struct tm* ptm;
      ptm = localtime(&time.tv_sec);
      char timeString[40];
      strftime(timeString, sizeof (timeString), "%Y-%m-%d %H:%M:%S", ptm);
      return std::string(timeString);
    }

    std::string Timestamp::getDate(double seconds) {
      struct timeval time;
      time.tv_sec = seconds;
      struct tm* ptm;
      ptm = localtime(&time.tv_sec);
      char timeString[40];
      strftime(timeString, sizeof (timeString), "%Y-%m-%d-%H-%M-%S", ptm);
      return std::string(timeString);
    }

    Timestamp::operator double() const {
      return mSeconds;
    }

    Timestamp::operator timespec() const {
      timespec time;
      time.tv_sec = (time_t)mSeconds;
      time.tv_nsec = (mSeconds - (time_t)mSeconds) * 1e9;
      return time;
    }

    bool Timestamp::operator == (const Timestamp& timestamp) const {
      return (mSeconds == timestamp.mSeconds);
    }

    bool Timestamp::operator == (double seconds) const {
      return (mSeconds == seconds);
    }

    bool Timestamp::operator != (const Timestamp& timestamp) const {
      return (mSeconds != timestamp.mSeconds);
    }

    bool Timestamp::operator != (double seconds) const {
      return (mSeconds != seconds);
    }

    bool Timestamp::operator > (const Timestamp& timestamp) const {
      return (mSeconds > timestamp.mSeconds);
    }

    bool Timestamp::operator > (double seconds) const {
      return (mSeconds > seconds);
    }

    bool Timestamp::operator < (const Timestamp& timestamp) const {
      return (mSeconds < timestamp.mSeconds);
    }

    bool Timestamp::operator < (double seconds) const {
      return (mSeconds < seconds);
    }

    bool Timestamp::operator >= (const Timestamp& timestamp) const {
      return (mSeconds >= timestamp.mSeconds);
    }

    bool Timestamp::operator >= (double seconds) const {
      return (mSeconds >= seconds);
    }

    bool Timestamp::operator <= (const Timestamp& timestamp) const {
      return (mSeconds <= timestamp.mSeconds);
    }

    bool Timestamp::operator <= (double seconds) const {
      return (mSeconds <= seconds);
    }

    Timestamp& Timestamp::operator += (double seconds) {
      mSeconds += seconds;
      return *this;
    }

    Timestamp& Timestamp::operator -= (double seconds) {
      mSeconds -= seconds;
      return *this;
    }

    double Timestamp::operator + (double seconds) const {
      return mSeconds + seconds;
    }

    double Timestamp::operator + (const Timestamp& timestamp) const {
      return mSeconds + timestamp.mSeconds;
    }

    double Timestamp::operator - (const Timestamp& timestamp) const {
      return mSeconds - timestamp.mSeconds;
    }

    double Timestamp::operator - (double seconds) const {
      return mSeconds - seconds;
    }

  }
}
