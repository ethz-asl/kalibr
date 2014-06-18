#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/python/operators.hpp>
#include <aslam/Time.hpp>
#include <sm/python/boost_serialization_pickle.hpp>
// static Time now();
// double toSec()  const { return (double)sec + 1e-9*(double)nsec; };
// T& fromSec(double t) { sec = (uint32_t)floor(t); nsec = (uint32_t)boost::math::round((t-sec) * 1e9);  return *static_cast<T*>(this);}

// double toSec() const { return (double)sec + 1e-9*(double)nsec; };
// int64_t toNSec() const {return (int64_t)sec*1000000000ll + (int64_t)nsec;  };
// T& fromSec(double t);

void exportTimeAndDuration() {
  using namespace aslam;
  using namespace boost::python;

  class_ < aslam::Time
      > ("Time", "A class to represent time. Initialize with Time(), Time(seconds), or Time(sec, nsec)")
          .def(init<>()).def(init<double>()).def(init<int, int>()).def(
          "now", &Time::now, "Get the current time").staticmethod("now").def(
          "toSec", &Time::toSec, "Convert the current time to seconds").def(
          "fromSec", &Time::fromSec,
          return_value_policy<copy_non_const_reference>(),
          "Set the time from seconds").def(
          "toNSec", &Time::toNSec, "Convert the current time to nanoseconds")
          .def("fromNSec", &Time::fromNSec,
               return_value_policy<copy_non_const_reference>(),
               "Set the time from nanoseconds")
      // These match the rospy time
          .def("to_sec", &Time::toSec, "Convert the current time to seconds")
          .def("from_sec", &Time::fromSec,
               return_value_policy<copy_non_const_reference>(),
               "Set the time from seconds").def(
          "to_nsec", &Time::toNSec, "Convert the current time to nanoseconds")

      .def(self - self).def(self + Duration()).def(self - Duration()).def(
          self += Duration()).def(self -= Duration()).def(self == self).def(
          self != self).def(self > self).def(self < self).def(self >= self).def(
          self <= self).def("isZero", &Time::isZero, "Is this time zero?")
          .def_readwrite("secs", &Time::sec).def_readwrite("nsecs", &Time::nsec)
          .def_readwrite("sec", &Time::sec).def_readwrite("nsec", &Time::nsec)
          .def_pickle(sm::python::pickle_suite<aslam::Time>())

      ;

  class_ < aslam::Duration
      > ("Duration", "A class to represent a time duration. Initialize with Duration(), Duration(seconds), or Duration(sec, nsec)")
          .def(init<>()).def(init<double>()).def(init<int, int>()).def(
          "toSec", &Duration::toSec, "Convert the current time to seconds").def(
          "fromSec", &Duration::fromSec,
          return_value_policy<copy_non_const_reference>(),
          "Set the time from seconds").def(
          "toNSec", &Duration::toNSec,
          "Convert the current time to nanoseconds").def(
          "fromNSec", &Duration::fromNSec,
          return_value_policy<copy_non_const_reference>(),
          "Set the time from nanoseconds").def(self - self).def(self + self).def(
          self += self).def(self -= self).def(self == self).def(self != self)
          .def(self > self).def(self < self).def(self >= self).def(self <= self)
          .def("isZero", &Duration::isZero, "Is this duration zero?").def_pickle(
          sm::python::pickle_suite<aslam::Duration>());

}
