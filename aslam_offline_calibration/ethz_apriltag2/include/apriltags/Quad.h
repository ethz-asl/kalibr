#ifndef QUAD_H
#define QUAD_H

#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "apriltags/Homography33.h"

namespace AprilTags {

class FloatImage;
class Segment;

using std::min;
using std::max;

//! Represents four segments that form a loop, and might be a tag.
class Quad {
public:
  static const int minimumEdgeLength = 6; //!< Minimum size of a tag (in pixels) as measured along edges and diagonals
  static float const maxQuadAspectRatio; //!< Early pruning of quads with insane ratios.

  //! Constructor
  /*! (x,y) are the optical center of the camera, which is
   *   needed to correctly compute the homography. */
  Quad(const std::vector< std::pair<float,float> >& p, const std::pair<float,float>& opticalCenter);

  //! Interpolate given that the lower left corner of the lower left cell is at (-1,-1) and the upper right corner of the upper right cell is at (1,1).
  std::pair<float,float> interpolate(float x, float y);

  //! Same as interpolate, except that the coordinates are interpreted between 0 and 1, instead of -1 and 1.
  std::pair<float,float> interpolate01(float x, float y);

  //! Points for the quad (in pixel coordinates), in counter clockwise order. These points are the intersections of segments.
  std::vector< std::pair<float,float> > quadPoints;

  //! Segments composing this quad
  std::vector<Segment*> segments;

  //! Total length (in pixels) of the actual perimeter observed for the quad.
  /*! This is in contrast to the geometric perimeter, some of which
   *  may not have been directly observed but rather inferred by
   *  intersecting segments. Quads with more observed perimeter are
   *  preferred over others. */
  float observedPerimeter;

  //! Given that the whole quad spans from (0,0) to (1,1) in "quad space", compute the pixel coordinates for a given point within that quad.
  /*!  Note that for most of the Quad's existence, we will not know the correct orientation of the tag. */
  Homography33 homography;

  //! Searches through a vector of Segments to form Quads.
  /*  @param quads any discovered quads will be added to this list
   *  @param path  the segments currently part of the search
   *  @param parent the first segment in the quad
   *  @param depth how deep in the search are we?
   */
  static void search(const FloatImage& fImage, std::vector<Segment*>& path,
                     Segment& parent, int depth, std::vector<Quad>& quads,
                     const std::pair<float,float>& opticalCenter);

#ifdef INTERPOLATE
 private:
  Eigen::Vector2f p0, p3, p01, p32;
#endif

};

} // namespace

#endif
