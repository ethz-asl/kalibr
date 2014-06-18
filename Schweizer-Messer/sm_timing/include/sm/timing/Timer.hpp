#ifndef SM_TIMER_HPP
#define SM_TIMER_HPP

#ifndef BOOST_DATE_TIME_NO_LOCALE
#define BOOST_DATE_TIME_NO_LOCALE
#include <boost/date_time/posix_time/posix_time.hpp>
#undef BOOST_DATE_TIME_NO_LOCALE
#else
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <map>
#include <vector>

#include <sm/assert_macros.hpp>

#ifdef _WIN32
#define SM_USE_HIGH_PERF_TIMER
#include <windows.h>
#endif


namespace sm {
namespace timing {
  
  SM_DEFINE_EXCEPTION(TimerException, std::runtime_error);
  
  struct TimerMapValue {
    // Initialize the window size for the rolling mean.
    TimerMapValue() : m_acc(boost::accumulators::tag::rolling_window::window_size = 50){}
    boost::accumulators::accumulator_set<
      double, 
      boost::accumulators::features<
	boost::accumulators::tag::lazy_variance,
	boost::accumulators::tag::sum,
	boost::accumulators::tag::min,
	boost::accumulators::tag::max,
	boost::accumulators::tag::rolling_mean,
	boost::accumulators::tag::mean
	>
      > m_acc;
  };
  
  // A class that has the timer interface but does nothing.
  // Swapping this in in place of the Timer class (say with a 
  // typedef) should allow one to disable timing. Because all
  // of the functions are inline, they should just disappear.
  class DummyTimer {
  public:
    DummyTimer(size_t /* handle */, bool /* constructStopped */ ){}
    DummyTimer(size_t /* handle */){}
    DummyTimer(std::string const & /* tag */){}
    DummyTimer(std::string const & /* tag */, bool /* constructStopped */){}
    ~DummyTimer(){}
    
    void start(){}
    void stop(){}
    bool isTiming(){ return false; }
  };
  
  class Timer {
  public:
    Timer(size_t handle, bool constructStopped = false);
    Timer(std::string const & tag, bool constructStopped = false);
    ~Timer();
    
    void start();
    void stop();
    bool isTiming();
  private:
#ifdef SM_USE_HIGH_PERF_TIMER
    LARGE_INTEGER m_time;
#else
    boost::posix_time::ptime m_time;
#endif
    bool m_timing;
    size_t m_handle;
  };
  
  class Timing{
  public:
    friend class Timer;
    // Static funcitons to query the timers:
    static  size_t getHandle(std::string const & tag);
    static  std::string getTag(size_t handle);
    static  double getTotalSeconds(size_t handle);
    static  double getTotalSeconds(std::string const & tag);
    static  double getMeanSeconds(size_t handle);
    static  double getMeanSeconds(std::string const & tag);
    static  size_t getNumSamples(size_t handle);
    static  size_t getNumSamples(std::string const & tag);
    static  double getVarianceSeconds(size_t handle);
    static  double getVarianceSeconds(std::string const & tag);
    static  double getMinSeconds(size_t handle);
    static  double getMinSeconds(std::string const & tag);
    static  double getMaxSeconds(size_t handle);
    static  double getMaxSeconds(std::string const & tag);
    static  double getHz(size_t handle);
    static  double getHz(std::string const & tag);
    static  void print(std::ostream & out);
    static  void reset(size_t handle);
    static  void reset(std::string const & tag);
    static  std::string print();
    static  std::string secondsToTimeString(double seconds);
    
  private:
    void addTime(size_t handle, double seconds);
    
    static Timing & instance();
    
    // Singleton design pattern
    Timing();
    ~Timing();
    
    typedef std::map<std::string,size_t> map_t;
    typedef std::vector<TimerMapValue> list_t;
    
    // Static members
    list_t m_timers;
    map_t m_tagMap;
#ifdef SM_USE_HIGH_PERF_TIMER
    double m_clockPeriod;
#endif
    size_t m_maxTagLength;
    
  }; // end class timer
  
#ifdef NDEBUG
  typedef DummyTimer DebugTimer;
#else
  typedef Timer DebugTimer;
#endif
  
} // namespace timing
} // end namespace sm

#endif // SM_TIMER_HPP
