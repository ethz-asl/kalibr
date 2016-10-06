#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/timing/NsecTimeUtilities.hpp>


void exportNsecTime() {
  using namespace boost::python;
  using namespace sm::timing;

  def("nsecNow", &nsecNow);
  /// \brief Get the epoch time as nanoseconds since the epoch.
  ///NsecTime nsecNow();
  
  /// \brief Convert the time (in integer nanoseconds) to decimal seconds.
  ///double nsecToSec( const NsecTime & time );
  def("nsecToSec", &nsecToSec);
  
  /// \brief Convert the time (in seconds) to integer nanoseconds
  ///NsecTime secToNsec( const double & time );
  def("secToNsec", &secToNsec);
}
