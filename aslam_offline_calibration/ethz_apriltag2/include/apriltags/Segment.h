#ifndef SEGMENT_H
#define SEGMENT_H

#include <cmath>
#include <vector>

namespace AprilTags {

//! Represents a line fit to a set of pixels whose gradients are similiar.
class Segment {
public:
  Segment();

  static int const minimumSegmentSize = 4; //!< Minimum number of pixels in a segment before we'll fit a line to it.
  static float const minimumLineLength; //!< In pixels. Calculated based on minimum plausible decoding size for Tag9 family.

  float getX0() const { return x0; }
  void setX0(float newValue) { x0 = newValue; }

  float getY0() const { return y0; }
  void setY0(float newValue) { y0 = newValue; }

  float getX1() const { return x1; }
  void setX1(float newValue) { x1 = newValue; }

  float getY1() const { return y1; }
  void setY1(float newValue) { y1 = newValue; }

  float getTheta() const { return theta; }
  void setTheta(float newValue) { theta = newValue; }

  float getLength() const { return length; }
  void setLength(float newValue) { length = newValue; }

  //! Returns the length of the Segment.
  float segmentLength();

  //! Print endpoint coordinates of this segment.
  void printSegment();

  //! ID of Segment.
  int getId() const { return segmentId; }

  std::vector<Segment*> children;

private:
  float x0, y0, x1, y1;
  float theta; // gradient direction (points towards white)
  float length; // length of line segment in pixels
  int segmentId;
  static int idCounter;
};

} // namsepace

#endif
