#include "apriltags/GLineSegment2D.h"
#include <limits>

namespace AprilTags {

GLineSegment2D::GLineSegment2D(const std::pair<float,float>& p0Arg, const std::pair<float,float>& p1Arg)
: line(p0Arg,p1Arg), p0(p0Arg), p1(p1Arg), weight() {}

GLineSegment2D GLineSegment2D::lsqFitXYW(const std::vector<XYWeight>& xyweight) {
	GLine2D gline = GLine2D::lsqFitXYW(xyweight);
	float maxcoord = -std::numeric_limits<float>::infinity();
	float mincoord = std::numeric_limits<float>::infinity();;
	
	for (unsigned int i = 0; i < xyweight.size(); i++) {
		std::pair<float,float> p(xyweight[i].x, xyweight[i].y);
		float coord = gline.getLineCoordinate(p);
		maxcoord = std::max(maxcoord, coord);
		mincoord = std::min(mincoord, coord);
	}
	
	std::pair<float,float> minValue = gline.getPointOfCoordinate(mincoord);
	std::pair<float,float> maxValue = gline.getPointOfCoordinate(maxcoord);
	return GLineSegment2D(minValue,maxValue);
}

} // namespace
