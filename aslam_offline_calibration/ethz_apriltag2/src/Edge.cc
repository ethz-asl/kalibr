#include "apriltags/Edge.h"
#include "apriltags/FloatImage.h"
#include "apriltags/MathUtil.h"
#include "apriltags/UnionFindSimple.h"

namespace AprilTags {

float const Edge::minMag = 0.004f;
float const Edge::maxEdgeCost = 30.f * float(M_PI) / 180.f;
int const Edge::WEIGHT_SCALE = 100;
float const Edge::thetaThresh = 100;
float const Edge::magThresh = 1200;

int Edge::edgeCost(float  theta0, float theta1, float mag1) {
  if (mag1 < minMag)  // mag0 was checked by the main routine so no need to recheck here
    return -1;

  const float thetaErr = std::abs(MathUtil::mod2pi(theta1 - theta0));
  if (thetaErr > maxEdgeCost)
    return -1;

  const float normErr = thetaErr / maxEdgeCost;
  return (int) (normErr*WEIGHT_SCALE);
}

void Edge::calcEdges(float theta0, int x, int y,
		     const FloatImage& theta, const FloatImage& mag,
		     std::vector<Edge> &edges, size_t &nEdges) {
  int width = theta.getWidth();
  int thisPixel = y*width+x;

  // horizontal edge
  int cost1 = edgeCost(theta0, theta.get(x+1,y), mag.get(x+1,y));
  if (cost1 >= 0) {
    edges[nEdges].cost = cost1;
    edges[nEdges].pixelIdxA = thisPixel;
    edges[nEdges].pixelIdxB = y*width+x+1;
    ++nEdges;
  }

  // vertical edge
  int cost2 = edgeCost(theta0, theta.get(x, y+1), mag.get(x,y+1));
  if (cost2 >= 0) {
    edges[nEdges].cost = cost2;
    edges[nEdges].pixelIdxA = thisPixel;
    edges[nEdges].pixelIdxB = (y+1)*width+x;
    ++nEdges;
  }
  
  // downward diagonal edge
  int cost3 = edgeCost(theta0, theta.get(x+1, y+1), mag.get(x+1,y+1));
  if (cost3 >= 0) {
    edges[nEdges].cost = cost3;
    edges[nEdges].pixelIdxA = thisPixel;
    edges[nEdges].pixelIdxB = (y+1)*width+x+1;
    ++nEdges;
  }

  // updward diagonal edge
  int cost4 = (x == 0) ? -1 : edgeCost(theta0, theta.get(x-1, y+1), mag.get(x-1,y+1));
  if (cost4 >= 0) {
    edges[nEdges].cost = cost4;
    edges[nEdges].pixelIdxA = thisPixel;
    edges[nEdges].pixelIdxB = (y+1)*width+x-1;
    ++nEdges;
  }
}

void Edge::mergeEdges(std::vector<Edge> &edges, UnionFindSimple &uf,
		      float tmin[], float tmax[], float mmin[], float mmax[]) {
  for (size_t i = 0; i < edges.size(); i++) {
    int ida = edges[i].pixelIdxA;
    int idb = edges[i].pixelIdxB;

    ida = uf.getRepresentative(ida);
    idb = uf.getRepresentative(idb);
      
    if (ida == idb)
      continue;

    int sza = uf.getSetSize(ida);
    int szb = uf.getSetSize(idb);

    float tmina = tmin[ida], tmaxa = tmax[ida];
    float tminb = tmin[idb], tmaxb = tmax[idb];

    float costa = (tmaxa-tmina);
    float costb = (tmaxb-tminb);

    // bshift will be a multiple of 2pi that aligns the spans of 'b' with 'a'
    // so that we can properly take the union of them.
    float bshift = MathUtil::mod2pi((tmina+tmaxa)/2, (tminb+tmaxb)/2) - (tminb+tmaxb)/2;

    float tminab = min(tmina, tminb + bshift);
    float tmaxab = max(tmaxa, tmaxb + bshift);

    if (tmaxab-tminab > 2*(float)M_PI) // corner case that's probably not too useful to handle correctly, oh well.
      tmaxab = tminab + 2*(float)M_PI;

    float mminab = min(mmin[ida], mmin[idb]);
    float mmaxab = max(mmax[ida], mmax[idb]);

    // merge these two clusters?
    float costab = (tmaxab - tminab);
    if (costab <= (min(costa, costb) + Edge::thetaThresh/(sza+szb)) &&
	(mmaxab-mminab) <= min(mmax[ida]-mmin[ida], mmax[idb]-mmin[idb]) + Edge::magThresh/(sza+szb)) {
	
      int idab = uf.connectNodes(ida, idb);
	
      tmin[idab] = tminab;
      tmax[idab] = tmaxab;
	
      mmin[idab] = mminab;
      mmax[idab] = mmaxab;
    }
  }
}

} // namespace
