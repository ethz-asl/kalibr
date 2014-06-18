#include "apriltags/GLine2D.h"

namespace AprilTags {

GLine2D::GLine2D() 
  : dx(0), dy(0), p(0,0), didNormalizeSlope(false), didNormalizeP(false) {}

GLine2D::GLine2D(float slope, float b) 
  : dx(1), dy(slope), p(0,b), didNormalizeSlope(false), didNormalizeP(false){}

GLine2D::GLine2D(float dX, float dY, const std::pair<float,float>& pt) 
  : dx(dX), dy(dY), p(pt), didNormalizeSlope(false), didNormalizeP(false) {}

GLine2D::GLine2D(const std::pair<float,float>& p1, const std::pair<float,float>& p2)
  : dx(p2.first - p1.first), dy(p2.second - p1.second), p(p1), didNormalizeSlope(false), didNormalizeP(false) {}

float GLine2D::getLineCoordinate(const std::pair<float,float>& pt) {
  normalizeSlope();
  return pt.first*dx + pt.second*dy;
}

std::pair<float,float> GLine2D::getPointOfCoordinate(float coord) {
  normalizeP();
  return std::pair<float,float>(p.first + coord*dx, p.second + coord*dy);
}

std::pair<float,float> GLine2D::intersectionWith(const GLine2D& line) const {
  float m00 = dx;
  float m01 = -line.getDx();
  float m10 = dy;
  float m11 = -line.getDy();

  // determinant of 'm'
  float det = m00*m11 - m01*m10;

  // parallel lines? if so, return (-1,0).
  if (fabs(det) < 1e-10)
    return std::pair<float,float>(-1,0);

  // inverse of 'm'
  float i00 = m11/det;
  // float i11 = m00/det;
  float i01 = -m01/det;
  // float i10 = -m10/det;

  float b00 = line.getFirst() - p.first;
  float b10 = line.getSecond() - p.second;

  float x00 = i00*b00 + i01*b10;

  return std::pair<float,float>(dx*x00+p.first, dy*x00+p.second);
}

GLine2D GLine2D::lsqFitXYW(const std::vector<XYWeight>& xyweights) {
  float Cxx=0, Cyy=0, Cxy=0, Ex=0, Ey=0, mXX=0, mYY=0, mXY=0, mX=0, mY=0;
  float n=0;

  int idx = 0;
  for (unsigned int i = 0; i < xyweights.size(); i++) {
    float x = xyweights[i].x;
    float y = xyweights[i].y;
    float alpha = xyweights[i].weight;

    mY  += y*alpha;
    mX  += x*alpha;
    mYY += y*y*alpha;
    mXX += x*x*alpha;
    mXY += x*y*alpha;
    n   += alpha;

    idx++;
  }
  
  Ex  = mX/n;
  Ey  = mY/n;
  Cxx = mXX/n - MathUtil::square(mX/n);
  Cyy = mYY/n - MathUtil::square(mY/n);
  Cxy = mXY/n - (mX/n)*(mY/n);

  // find dominant direction via SVD
  float phi = 0.5f*std::atan2(-2*Cxy,(Cyy-Cxx));
  // float rho = Ex*cos(phi) + Ey*sin(phi); //why is this needed if he never uses it?
  std::pair<float,float> pts = std::pair<float,float>(Ex,Ey);

  // compute line parameters
	return GLine2D(-std::sin(phi), std::cos(phi), pts);
}

void GLine2D::normalizeSlope() {
  if ( !didNormalizeSlope ) {
    float mag = std::sqrt(dx*dx+dy*dy);
    dx /= mag;
    dy /= mag;
    didNormalizeSlope=true;
  }
}

void GLine2D::normalizeP() {
  if ( !didNormalizeP ) {
    normalizeSlope();
    // we already have a point (P) on the line, and we know the line vector U
    // and its perpendicular vector V: so, P'=P.*V *V
    float dotprod = -dy*p.first + dx*p.second;
    p = std::pair<float,float>(-dy*dotprod, dx*dotprod);
    didNormalizeP = true;
  }
}

} // namespace
