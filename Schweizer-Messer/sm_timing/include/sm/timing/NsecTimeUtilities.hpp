/**
 * @file   NsecTimeUtilities.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>
 * @date   Sat Jul 20 12:39:54 2013
 * 
 * @brief  Functions to support the use of nanosecond epoch time.
 * 
 */

#include <chrono>
#include <boost/cstdint.hpp>

namespace sm {
namespace timing {

/// \brief Nanoseconds since the epoch.
typedef boost::int64_t NsecTime;

/// \brief Convert nanoseconds since the epoch to std::chrono
std::chrono::system_clock::time_point nsecToChrono( const NsecTime & time );

/// \brief Convert std::chrono to nanoseconds since the epoch.
NsecTime chronoToNsec( const std::chrono::system_clock::time_point & time );

/// \brief Get the epoch time as nanoseconds since the epoch.
NsecTime nsecNow();

/// \brief Convert the time (in integer nanoseconds) to decimal seconds.
double nsecToSec( const NsecTime & time );

/// \brief Convert the time (in seconds) to integer nanoseconds
NsecTime secToNsec( const double & time );

} // namespace timing
} // namespace sm
