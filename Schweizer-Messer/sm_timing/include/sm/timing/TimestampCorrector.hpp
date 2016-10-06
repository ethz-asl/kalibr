#include <vector>
#include <sm/assert_macros.hpp>

namespace sm {
  namespace timing {
    
    /**
     * \class TimestampCorrector
     *
     * An implementation of the convex hull algorithm for one-way
     * timestamp synchronization from 
     *
     * L. Zhang, Z. Liu, and C. Honghui Xia,
     * “Clock synchronization algorithms for network measurements”,
     * in INFOCOM 2002. Twenty-First Annual Joint Conference of the
     * IEEE Computer and Communications Societies., vol. 1. IEEE,
     * 2002, pp. 160–169 vol.1.
     * 
     */
    template<typename TIME_T>
    class TimestampCorrector
    {
    public:
      typedef TIME_T time_t;

      SM_DEFINE_EXCEPTION(Exception, std::runtime_error);
      SM_DEFINE_EXCEPTION(TimeWentBackwardsException, Exception);
      SM_DEFINE_EXCEPTION(NotInitializedException, Exception);

      
      TimestampCorrector();
      virtual ~TimestampCorrector();

      /** 
       * Get an estimate of the local time of a given measurement
       * from the remote timestamp, the local timestamp, and the 
       * previous history of timings.
       *
       * NOTE: this function must be called with monotonically increasing
       *       remote timestamps. If this is not followed, an exception will
       *       be thrown.
       * 
       * @param remoteTime The time of an event on the remote clock
       * @param localTime  The timestamp that the event was received locally
       * 
       * @return The estimated actual local time of the event
       */
      time_t correctTimestamp(const time_t & remoteTime, const time_t & localTime);
      
      /** 
       * Using the current best estimate of the relationship between
       * remote and local clocks, get the local time of a remote timestamp.
       * 
       * @param remoteTime The time of an event on the remote clock.
       * 
       * @return The estimated local time of the event.
       */
      time_t getLocalTime(const time_t & remoteTime) const;

      /** 
       * @return The number of points in the convex hull
       */
      size_t convexHullSize() const { return _convexHull.size(); }

      double getSlope() const;
      double getOffset() const;
      
      void printHullPoints()
      {
	for(unsigned i = 0; i < _convexHull.size(); ++i)
	  {
	    std::cout << i << "\t" << _convexHull[i].x << "\t" << _convexHull[i].y;
	    if(i == _midpointSegmentIndex)
	      std::cout << " <<< Midpoint segment start";
	    std::cout << std::endl;
	  }
      }
    private:
      
      class Point
      {
      public:
	Point(const time_t & x, const time_t & y) : x(x), y(y) {}
	// remote time
	time_t x;
	// local time
	time_t y;

	Point operator-(const Point & p) const { return Point(x - p.x, y - p.y); }
	Point operator+(const Point & p) const { return Point(x + p.x, y + p.y); }
	bool operator<(const Point & p) const { return x < p.x; }
	bool operator<(const time_t & t) const { return x < t; }
      };

      /** 
       * Is the point above the line defined by the top two points of
       * the convex hull?
       * 
       * @param p the point to check
       * 
       * @return true if the point is above the line.
       */
      bool isAboveTopLine( const Point & p ) const;

      /** 
       * Is the point, p, above the line defined by the points l1 and l2?
       * 
       * @param l1 
       * @param l2 
       * @param p 
       * 
       * @return 
       */
      bool isAboveLine( const Point & l1, const Point & l2, const Point & p ) const;

      typedef std::vector< Point > convex_hull_t;
      convex_hull_t _convexHull;

      size_t _midpointSegmentIndex;
      
    };

  } // namespace timing
} // namespace sm

#include "implementation/TimestampCorrector.hpp"
