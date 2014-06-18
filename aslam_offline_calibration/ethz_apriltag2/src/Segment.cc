#include "apriltags/Segment.h"
#include <iostream>

namespace AprilTags {

const float Segment::minimumLineLength = 4;

Segment::Segment() 
  : children(), x0(0), y0(0), x1(0), y1(0), theta(0), length(0), segmentId(++idCounter) {}

float Segment::segmentLength() {
  return std::sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
}

void Segment::printSegment() {
  std::cout <<"("<< x0 <<","<< y0 <<"), "<<"("<< x1 <<","<< y1 <<")" << std::endl;
}

int Segment::idCounter = 0;

} // namespace
