#include <Eigen/Dense>

#include "apriltags/FloatImage.h"
#include "apriltags/MathUtil.h"
#include "apriltags/GLine2D.h"
#include "apriltags/Quad.h"
#include "apriltags/Segment.h"

namespace AprilTags {
	
const float Quad::maxQuadAspectRatio = 32;

Quad::Quad(const std::vector< std::pair<float,float> >& p, const std::pair<float,float>& opticalCenter)
  : quadPoints(p), segments(), observedPerimeter(), homography(opticalCenter) {
#ifdef STABLE_H
  std::vector< std::pair<float,float> > srcPts;
  srcPts.push_back(std::make_pair(-1, -1));
  srcPts.push_back(std::make_pair( 1, -1));
  srcPts.push_back(std::make_pair( 1,  1));
  srcPts.push_back(std::make_pair(-1,  1));
  homography.setCorrespondences(srcPts, p);
#else
  homography.addCorrespondence(-1, -1, quadPoints[0].first, quadPoints[0].second);
  homography.addCorrespondence( 1, -1, quadPoints[1].first, quadPoints[1].second);
  homography.addCorrespondence( 1,  1, quadPoints[2].first, quadPoints[2].second);
  homography.addCorrespondence(-1,  1, quadPoints[3].first, quadPoints[3].second);
#endif

#ifdef INTERPOLATE
  p0 = Eigen::Vector2f(p[0].first, p[0].second);
  p3 = Eigen::Vector2f(p[3].first, p[3].second);
  p01 = (Eigen::Vector2f(p[1].first, p[1].second) - p0);
  p32 = (Eigen::Vector2f(p[2].first, p[2].second) - p3);
#endif
}

std::pair<float,float> Quad::interpolate(float x, float y) {
#ifdef INTERPOLATE
  Eigen::Vector2f r1 = p0 + p01 * (x+1.)/2.;
  Eigen::Vector2f r2 = p3 + p32 * (x+1.)/2.;
  Eigen::Vector2f r = r1 + (r2-r1) * (y+1)/2;
  return std::pair<float,float>(r(0), r(1));
#else
  return homography.project(x,y);
#endif
}

std::pair<float,float> Quad::interpolate01(float x, float y) {
  return interpolate(2*x-1, 2*y-1);
}

void Quad::search(const FloatImage& fImage, std::vector<Segment*>& path,
                  Segment& parent, int depth, std::vector<Quad>& quads,
                  const std::pair<float,float>& opticalCenter) {
  // cout << "Searching segment " << parent.getId() << ", depth=" << depth << ", #children=" << parent.children.size() << endl;
  // terminal depth occurs when we've found four segments.
  if (depth == 4) {
    // cout << "Entered terminal depth" << endl; // debug code

    // Is the first segment the same as the last segment (i.e., a loop?)
    if (path[4] == path[0]) {
      // the 4 corners of the quad as computed by the intersection of segments.
      std::vector< std::pair<float,float> > p(4);
      float calculatedPerimeter = 0;
      bool bad = false;
      for (int i = 0; i < 4; i++) {
	// compute intersections between all the lines. This will give us 
	// sub-pixel accuracy for the corners of the quad.
	GLine2D linea(std::make_pair(path[i]->getX0(),path[i]->getY0()),
		      std::make_pair(path[i]->getX1(),path[i]->getY1()));
	GLine2D lineb(std::make_pair(path[i+1]->getX0(),path[i+1]->getY0()),
		      std::make_pair(path[i+1]->getX1(),path[i+1]->getY1()));

	p[i] = linea.intersectionWith(lineb);
	calculatedPerimeter += path[i]->getLength();

	// no intersection? Occurs when the lines are almost parallel.
	if (p[i].first == -1)
	  bad = true;
      }
      // cout << "bad = " << bad << endl;
      // eliminate quads that don't form a simply connected loop, i.e., those 
      // that form an hour glass, or wind the wrong way.
      if (!bad) {
	float t0 = std::atan2(p[1].second-p[0].second, p[1].first-p[0].first);
	float t1 = std::atan2(p[2].second-p[1].second, p[2].first-p[1].first);
	float t2 = std::atan2(p[3].second-p[2].second, p[3].first-p[2].first);
	float t3 = std::atan2(p[0].second-p[3].second, p[0].first-p[3].first);

	//	double ttheta = fmod(t1-t0, 2*M_PI) + fmod(t2-t1, 2*M_PI) +
	//	  fmod(t3-t2, 2*M_PI) + fmod(t0-t3, 2*M_PI);
	float ttheta = MathUtil::mod2pi(t1-t0) + MathUtil::mod2pi(t2-t1) +
	  MathUtil::mod2pi(t3-t2) + MathUtil::mod2pi(t0-t3);
	// cout << "ttheta=" << ttheta << endl;
	// the magic value is -2*PI. It should be exact, 
	// but we allow for (lots of) numeric imprecision.
	if (ttheta < -7 || ttheta > -5)
	  bad = true;
      }

      if (!bad) {
	float d0 = MathUtil::distance2D(p[0], p[1]);
	float d1 = MathUtil::distance2D(p[1], p[2]);
	float d2 = MathUtil::distance2D(p[2], p[3]);
	float d3 = MathUtil::distance2D(p[3], p[0]);
	float d4 = MathUtil::distance2D(p[0], p[2]);
	float d5 = MathUtil::distance2D(p[1], p[3]);

	// check sizes
	if (d0 < Quad::minimumEdgeLength || d1 < Quad::minimumEdgeLength || d2 < Quad::minimumEdgeLength ||
	    d3 < Quad::minimumEdgeLength || d4 < Quad::minimumEdgeLength || d5 < Quad::minimumEdgeLength) {
	  bad = true;
	  // cout << "tagsize too small" << endl;
	}

	// check aspect ratio
	float dmax = max(max(d0,d1), max(d2,d3));
	float dmin = min(min(d0,d1), min(d2,d3));

	if (dmax > dmin*Quad::maxQuadAspectRatio) {
	  bad = true;
	  // cout << "aspect ratio too extreme" << endl;
	}
      }

      if (!bad) {
	Quad q(p, opticalCenter);
	q.segments=path;
	q.observedPerimeter = calculatedPerimeter;
	quads.push_back(q);
      }
    }
    return;
  }

  //  if (depth >= 1) // debug code
  //cout << "depth: " << depth << endl;

  // Not terminal depth. Recurse on any children that obey the correct handedness.
  for (unsigned int i = 0; i < parent.children.size(); i++) {
    Segment &child = *parent.children[i];
    //    cout << "  Child " << child.getId() << ":  ";
    // (handedness was checked when we created the children)
    
    // we could rediscover each quad 4 times (starting from
    // each corner). If we had an arbitrary ordering over
    // points, we can eliminate the redundant detections by
    // requiring that the first corner have the lowest
    // value. We're arbitrarily going to use theta...
    if ( child.getTheta() > path[0]->getTheta() ) {
      // cout << "theta failed: " << child.getTheta() << " > " << path[0]->getTheta() << endl;
      continue;
    }
    path[depth+1] = &child;
    search(fImage, path, child, depth+1, quads, opticalCenter);
  }
}

} // namespace
