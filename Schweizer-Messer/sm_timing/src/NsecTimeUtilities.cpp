#include <sm/timing/NsecTimeUtilities.hpp>

namespace sm {
namespace timing {

/// \brief Convert nanoseconds since the epoch to std::chrono
std::chrono::system_clock::time_point nsecToChrono( const NsecTime & time ) {
  std::chrono::nanoseconds tt(time);
  std::chrono::system_clock::time_point tp(std::chrono::duration_cast<std::chrono::system_clock::duration>(tt));
  return tp;
}


/// \brief Convert std::chrono to nanoseconds since the epoch.
NsecTime chronoToNsec( const std::chrono::system_clock::time_point & time ) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>( time.time_since_epoch()).count();
}


/// \brief Get the epoch time as nanoseconds since the epoch.
NsecTime nsecNow() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>( std::chrono::system_clock::now().time_since_epoch() ).count();

}


/// \brief Convert the time (in integer nanoseconds) to decimal seconds.
double nsecToSec( const NsecTime & time ) {
  return (double) time * 1e-9;
}


/// \brief Convert the time (in seconds) to integer nanoseconds
NsecTime secToNsec( const double & time ) {
  return boost::int64_t( time * 1e9 );
}

} // namespace timing
} // namespace sm
