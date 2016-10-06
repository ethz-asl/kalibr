

namespace sm {
namespace timing {
    
template<typename T>
TimestampCorrector<T>::TimestampCorrector() : _midpointSegmentIndex(0)
{
      
}

template<typename T>
TimestampCorrector<T>::~TimestampCorrector()
{

}


// Returns the local time
template<typename T>
typename TimestampCorrector<T>::time_t TimestampCorrector<T>::correctTimestamp(const time_t & remoteTime, const time_t & localTime)
{
  // Make sure this point is forward in time.
  if(!_convexHull.empty())
  {
    SM_ASSERT_GT(TimeWentBackwardsException, remoteTime, _convexHull[_convexHull.size() - 1].x, "The correction algorithm requires that times are passed in with monotonically increasing remote timestamps");
  }

  Point p(remoteTime, localTime);

  // If the point is not above the top line in the stack      
  if( !isAboveTopLine( p ) )
  {
    // While on the top of the stack points are above a line between two back and the new point... 
    while(_convexHull.size() >= 2 && isAboveLine(_convexHull[_convexHull.size() - 2], p, _convexHull[_convexHull.size() - 1]) )
    {
      _convexHull.pop_back();
    }
  }

  // In either case, push the new point on to the convex hull
  _convexHull.push_back(p);

  // Update the midpoint pointer...
  if(_convexHull.size() >= 3)
  {
    T midpoint = (_convexHull[0].x + remoteTime) / 2.0;
	  
    typename convex_hull_t::iterator lbit = std::lower_bound(_convexHull.begin(), _convexHull.end(), midpoint);
    _midpointSegmentIndex = lbit - _convexHull.begin() - 1;
    SM_ASSERT_LT_DBG(Exception, _midpointSegmentIndex, _convexHull.size() - 1, "The computed midpoint segment is out of bounds. Elements in hull: " 
                     << _convexHull.size() << ", Start time: " << _convexHull[0].x << ", End time: " << _convexHull[_convexHull.size() - 1].x 
                     << ", midpoint: " << midpoint);

    SM_ASSERT_GE_DBG(Exception, midpoint, _convexHull[_midpointSegmentIndex].x, "The computed midpoint is not within the midpoint segment");
    SM_ASSERT_LE_DBG(Exception, midpoint, _convexHull[_midpointSegmentIndex + 1].x, "The computed midpoint is not within the midpoint segment");
  }
  else
  {
    // and if there aren't enough data points, just return the sampled local time.
    return localTime;
  }

  return getLocalTime(remoteTime);
      
}

template<typename T>
double TimestampCorrector<T>::getSlope() const {
  SM_ASSERT_GE(NotInitializedException, _convexHull.size(), 2, "The timestamp correction requires at least two data points before this funciton can be called");

  // Get the line at the time midpoint.
  const Point & l1 = _convexHull[_midpointSegmentIndex];
  const Point & l2 = _convexHull[_midpointSegmentIndex + 1];

  // look up the local timestamp
  return  double(l2.y - l1.y) / double(l2.x - l1.x); 
}

template<typename T>
double TimestampCorrector<T>::getOffset() const {
  SM_ASSERT_GE(NotInitializedException, _convexHull.size(), 2, "The timestamp correction requires at least two data points before this funciton can be called");
  // Get the line at the time midpoint.
  const Point & l1 = _convexHull[_midpointSegmentIndex];
  const Point & l2 = _convexHull[_midpointSegmentIndex + 1];
  
  // look up the local timestamp
  return  double(l1.y) + (double(-l1.x) * double(l2.y - l1.y) / double(l2.x - l1.x) ); 
}

  
// Get the local time from the remote time.
template<typename T>
typename TimestampCorrector<T>::time_t TimestampCorrector<T>::getLocalTime(const time_t & remoteTime) const
{
  SM_ASSERT_GE(NotInitializedException, _convexHull.size(), 2, "The timestamp correction requires at least two data points before this funciton can be called");

  // Get the line at the time midpoint.
  const Point & l1 = _convexHull[_midpointSegmentIndex];
  const Point & l2 = _convexHull[_midpointSegmentIndex + 1];

  // look up the local timestamp
  return  l1.y + ((remoteTime - l1.x) * (l2.y - l1.y) / (l2.x - l1.x) ); 
}

template<typename T>
bool TimestampCorrector<T>::isAboveTopLine(const Point & p) const
{
  if(_convexHull.size() < 2)
  {
    return true;
  }
            
  return isAboveLine(_convexHull[ _convexHull.size() - 2 ],
                     _convexHull[ _convexHull.size() - 1],
                     p);
      
      
}

template<typename T>
bool TimestampCorrector<T>::isAboveLine(const Point & l1, const Point & l2, const Point & p) const
{
  Point v1 = l2 - l1;
  Point v2 = p - l1;
      
  T determinant = v1.x * v2.y - v1.y * v2.x;

  return determinant >= 0.0;
}




} // namespace timing
} // namespace sm
