#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/timing/TimestampCorrector.hpp>
#include <boost/cstdint.hpp>

template<typename TIME_T>
void exportTimestampCorrector(const std::string & className)
{
  using namespace boost::python;
  using namespace sm::timing;
  
  class_< TimestampCorrector<TIME_T> >( className.c_str(), init<>() )
    .def("correctTimestamp", &TimestampCorrector<TIME_T>::correctTimestamp, "correctedEventLocalTime = correctTimestamp(eventRemoteTime, eventLocalTimes).\nNote: This function must be called with monotonically increasing remote timestamps.")
    .def("getLocalTime", &TimestampCorrector<TIME_T>::getLocalTime, "eventLocalTime = getLocalTime(eventRemoteTime)")
    .def("convexHullSize", &TimestampCorrector<TIME_T>::convexHullSize)
    .def("printHullPoints", &TimestampCorrector<TIME_T>::printHullPoints)
      .def("getSlope", &TimestampCorrector<TIME_T>::getSlope)
      .def("getOffset", &TimestampCorrector<TIME_T>::getOffset)
    ;

}


void exportTimestampCorrectors()
{
  exportTimestampCorrector<double>("DoubleTimestampCorrector");
  exportTimestampCorrector<boost::int64_t>("LongTimestampCorrector");
}
