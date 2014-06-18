#include <algorithm>
#include <cmath>
#include <climits>
#include <map>
#include <vector>
#include <iostream>

#include <Eigen/Dense>

#include "apriltags/Edge.h"
#include "apriltags/FloatImage.h"
#include "apriltags/Gaussian.h"
#include "apriltags/GrayModel.h"
#include "apriltags/GLine2D.h"
#include "apriltags/GLineSegment2D.h"
#include "apriltags/Gridder.h"
#include "apriltags/Homography33.h"
#include "apriltags/MathUtil.h"
#include "apriltags/Quad.h"
#include "apriltags/Segment.h"
#include "apriltags/TagFamily.h"
#include "apriltags/UnionFindSimple.h"
#include "apriltags/XYWeight.h"

#include "apriltags/TagDetector.h"

//#define DEBUG_APRIL

#ifdef DEBUG_APRIL
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

using namespace std;

namespace AprilTags {

  std::vector<TagDetection> TagDetector::extractTags(const cv::Mat& image) {

    // convert to internal AprilTags image (todo: slow, change internally to OpenCV)
    int width = image.cols;
    int height = image.rows;
    AprilTags::FloatImage fimOrig(width, height);
    int i = 0;
    for (int y=0; y<height; y++) {
      for (int x=0; x<width; x++) {
        fimOrig.set(x, y, image.data[i]/255.);
        i++;
      }
    }
    std::pair<int,int> opticalCenter(width/2, height/2);

#ifdef DEBUG_APRIL
#if 0
  { // debug - write
    int height_ = fimOrig.getHeight();
    int width_  = fimOrig.getWidth();
    cv::Mat image(height_, width_, CV_8UC3);
    {
      for (int y=0; y<height_; y++) {
        for (int x=0; x<width_; x++) {
          cv::Vec3b v;
          //        float vf = fimMag.get(x,y);
          float vf = fimOrig.get(x,y);
          int val = (int)(vf * 255.);
          if ((val & 0xffff00) != 0) {printf("problem... %i\n", val);}
          for (int k=0; k<3; k++) {
            v(k) = val;
          }
          image.at<cv::Vec3b>(y, x) = v;
        }
      }
    }
    imwrite("out.bmp", image);
  }
#endif
#if 0
  FloatImage fimOrig = fimOrig_;
  { // debug - read

    cv::Mat image = cv::imread("test.bmp");
    int height_ = fimOrig.getHeight();
    int width_  = fimOrig.getWidth();
    {
      for (int y=0; y<height_; y++) {
        for (int x=0; x<width_; x++) {
          cv::Vec3b v = image.at<cv::Vec3b>(y,x);
          float val = (float)v(0)/255.;
          fimOrig.set(x,y,val);
        }
      }
    }
  }
#endif
#endif

  //================================================================
  // Step one: preprocess image (convert to grayscale) and low pass if necessary

  FloatImage fim = fimOrig;
  
  //! Gaussian smoothing kernel applied to image (0 == no filter).
  /*! Used when sampling bits. Filtering is a good idea in cases
   * where A) a cheap camera is introducing artifical sharpening, B)
   * the bayer pattern is creating artifcats, C) the sensor is very
   * noisy and/or has hot/cold pixels. However, filtering makes it
   * harder to decode very small tags. Reasonable values are 0, or
   * [0.8, 1.5].
   */
  float sigma = 0;

  //! Gaussian smoothing kernel applied to image (0 == no filter).
  /*! Used when detecting the outline of the box. It is almost always
   * useful to have some filtering, since the loss of small details
   * won't hurt. Recommended value = 0.8. The case where sigma ==
   * segsigma has been optimized to avoid a redundant filter
   * operation.
   */
  float segSigma = 0.8f;

  if (sigma > 0) {
    int filtsz = ((int) max(3.0f, 3*sigma)) | 1;
    std::vector<float> filt = Gaussian::makeGaussianFilter(sigma, filtsz);
    fim.filterFactoredCentered(filt, filt);
  }

  //================================================================
  // Step two: Compute the local gradient. We store the direction and magnitude.
  // This step is quite sensitve to noise, since a few bad theta estimates will
  // break up segments, causing us to miss Quads. It is useful to do a Gaussian
  // low pass on this step even if we don't want it for encoding.

  FloatImage fimSeg;
  if (segSigma > 0) {
    if (segSigma == sigma) {
      fimSeg = fim;
    } else {
      // blur anew
      int filtsz = ((int) max(3.0f, 3*segSigma)) | 1;
      std::vector<float> filt = Gaussian::makeGaussianFilter(segSigma, filtsz);
      fimSeg = fimOrig;
      fimSeg.filterFactoredCentered(filt, filt);
    }
  } else {
    fimSeg = fimOrig;
  }

  FloatImage fimTheta(fimSeg.getWidth(), fimSeg.getHeight());
  FloatImage fimMag(fimSeg.getWidth(), fimSeg.getHeight());
  

  #pragma omp parallel for
  for (int y = 1; y < fimSeg.getHeight()-1; y++) {
    for (int x = 1; x < fimSeg.getWidth()-1; x++) {
      float Ix = fimSeg.get(x+1, y) - fimSeg.get(x-1, y);
      float Iy = fimSeg.get(x, y+1) - fimSeg.get(x, y-1);

      float mag = Ix*Ix + Iy*Iy;
#if 0 // kaess: fast version, but maybe less accurate?
      float theta = MathUtil::fast_atan2(Iy, Ix);
#else
      float theta = atan2(Iy, Ix);
#endif
      
      fimTheta.set(x, y, theta);
      fimMag.set(x, y, mag);
    }
  }

#ifdef DEBUG_APRIL
  int height_ = fimSeg.getHeight();
  int width_  = fimSeg.getWidth();
  cv::Mat image(height_, width_, CV_8UC3);
  {
    for (int y=0; y<height_; y++) {
      for (int x=0; x<width_; x++) {
        cv::Vec3b v;
        //        float vf = fimMag.get(x,y);
        float vf = fimOrig.get(x,y);
        int val = (int)(vf * 255.);
        if ((val & 0xffff00) != 0) {printf("problem... %i\n", val);}
        for (int k=0; k<3; k++) {
          v(k) = val;
        }
        image.at<cv::Vec3b>(y, x) = v;
      }
    }
  }
#endif

  //================================================================
  // Step three. Extract edges by grouping pixels with similar
  // thetas together. This is a greedy algorithm: we start with
  // the most similar pixels.  We use 4-connectivity.
  UnionFindSimple uf(fimSeg.getWidth()*fimSeg.getHeight());
  
  vector<Edge> edges(width*height*4);
  size_t nEdges = 0;

  // Bounds on the thetas assigned to this group. Note that because
  // theta is periodic, these are defined such that the average
  // value is contained *within* the interval.
  { // limit scope of storage
    /* Previously all this was on the stack, but this is 1.2MB for 320x240 images
     * That's already a problem for OS X (default 512KB thread stack size),
     * could be a problem elsewhere for bigger images... so store on heap */
    vector<float> storage(width*height*4);  // do all the memory in one big block, exception safe
    float * tmin = &storage[width*height*0];
    float * tmax = &storage[width*height*1];
    float * mmin = &storage[width*height*2];
    float * mmax = &storage[width*height*3];
                  
    for (int y = 0; y+1 < height; y++) {
      for (int x = 0; x+1 < width; x++) {
                                  
        float mag0 = fimMag.get(x,y);
        if (mag0 < Edge::minMag)
          continue;
        mmax[y*width+x] = mag0;
        mmin[y*width+x] = mag0;
                                  
        float theta0 = fimTheta.get(x,y);
        tmin[y*width+x] = theta0;
        tmax[y*width+x] = theta0;
                                  
        // Calculates then adds edges to 'vector<Edge> edges'
        Edge::calcEdges(theta0, x, y, fimTheta, fimMag, edges, nEdges);
                                  
        // XXX Would 8 connectivity help for rotated tags?
        // Probably not much, so long as input filtering hasn't been disabled.
      }
    }
                  
    edges.resize(nEdges);
    std::stable_sort(edges.begin(), edges.end());
    Edge::mergeEdges(edges,uf,tmin,tmax,mmin,mmax);
  }
          
  //================================================================
  // Step four: Loop over the pixels again, collecting statistics for each cluster.
  // We will soon fit lines (segments) to these points.

  map<int, vector<XYWeight> > clusters;
  for (int y = 0; y+1 < fimSeg.getHeight(); y++) {
    for (int x = 0; x+1 < fimSeg.getWidth(); x++) {
      if (uf.getSetSize(y*fimSeg.getWidth()+x) < Segment::minimumSegmentSize)
	continue;

      int rep = (int) uf.getRepresentative(y*fimSeg.getWidth()+x);
     
      map<int, vector<XYWeight> >::iterator it = clusters.find(rep);
      if ( it == clusters.end() ) {
	clusters[rep] = vector<XYWeight>();
	it = clusters.find(rep);
      }
      vector<XYWeight> &points = it->second;
      points.push_back(XYWeight(x,y,fimMag.get(x,y)));
    }
  }

  //================================================================
  // Step five: Loop over the clusters, fitting lines (which we call Segments).
  std::vector<Segment> segments; //used in Step six
  std::map<int, std::vector<XYWeight> >::const_iterator clustersItr;
  for (clustersItr = clusters.begin(); clustersItr != clusters.end(); clustersItr++) {
    std::vector<XYWeight> points = clustersItr->second;
    GLineSegment2D gseg = GLineSegment2D::lsqFitXYW(points);

    // filter short lines
    float length = MathUtil::distance2D(gseg.getP0(), gseg.getP1());
    if (length < Segment::minimumLineLength)
      continue;

    Segment seg;
    float dy = gseg.getP1().second - gseg.getP0().second;
    float dx = gseg.getP1().first - gseg.getP0().first;

    float tmpTheta = std::atan2(dy,dx);

    seg.setTheta(tmpTheta);
    seg.setLength(length);

    // We add an extra semantic to segments: the vector
    // p1->p2 will have dark on the left, white on the right.
    // To do this, we'll look at every gradient and each one
    // will vote for which way they think the gradient should
    // go. This is way more retentive than necessary: we
    // could probably sample just one point!

    float flip = 0, noflip = 0;
    for (unsigned int i = 0; i < points.size(); i++) {
      XYWeight xyw = points[i];
      
      float theta = fimTheta.get((int) xyw.x, (int) xyw.y);
      float mag = fimMag.get((int) xyw.x, (int) xyw.y);

      // err *should* be +M_PI/2 for the correct winding, but if we
      // got the wrong winding, it'll be around -M_PI/2.
      float err = MathUtil::mod2pi(theta - seg.getTheta());

      if (err < 0)
	noflip += mag;
      else
	flip += mag;
    }

    if (flip > noflip) {
      float temp = seg.getTheta() + (float)M_PI;
      seg.setTheta(temp);
    }

    float dot = dx*std::cos(seg.getTheta()) + dy*std::sin(seg.getTheta());
    if (dot > 0) {
      seg.setX0(gseg.getP1().first); seg.setY0(gseg.getP1().second);
      seg.setX1(gseg.getP0().first); seg.setY1(gseg.getP0().second);
    }
    else {
      seg.setX0(gseg.getP0().first); seg.setY0(gseg.getP0().second);
      seg.setX1(gseg.getP1().first); seg.setY1(gseg.getP1().second);
    }

    segments.push_back(seg);
  }

#ifdef DEBUG_APRIL
#if 0
  {
    for (vector<Segment>::iterator it = segments.begin(); it!=segments.end(); it++) {
      long int r = random();
      cv::line(image,
               cv::Point2f(it->getX0(), it->getY0()),
               cv::Point2f(it->getX1(), it->getY1()),
               cv::Scalar(r%0xff,(r%0xff00)>>8,(r%0xff0000)>>16,0) );
    }
  }
#endif
#endif

  // Step six: For each segment, find segments that begin where this segment ends.
  // (We will chain segments together next...) The gridder accelerates the search by
  // building (essentially) a 2D hash table.
  Gridder<Segment> gridder(0,0,width,height,10);
  
  // add every segment to the hash table according to the position of the segment's
  // first point. Remember that the first point has a specific meaning due to our
  // left-hand rule above.
  for (unsigned int i = 0; i < segments.size(); i++) {
    gridder.add(segments[i].getX0(), segments[i].getY0(), &segments[i]);
  }
  
  // Now, find child segments that begin where each parent segment ends.
  for (unsigned i = 0; i < segments.size(); i++) {
    Segment &parentseg = segments[i];
      
    //compute length of the line segment
    GLine2D parentLine(std::pair<float,float>(parentseg.getX0(), parentseg.getY0()),
		       std::pair<float,float>(parentseg.getX1(), parentseg.getY1()));

    Gridder<Segment>::iterator iter = gridder.find(parentseg.getX1(), parentseg.getY1(), 0.5f*parentseg.getLength());
    while(iter.hasNext()) {
      Segment &child = iter.next();
      if (MathUtil::mod2pi(child.getTheta() - parentseg.getTheta()) > 0) {
	continue;
      }

      // compute intersection of points
      GLine2D childLine(std::pair<float,float>(child.getX0(), child.getY0()),
			std::pair<float,float>(child.getX1(), child.getY1()));

      std::pair<float,float> p = parentLine.intersectionWith(childLine);
      if (p.first == -1) {
	continue;
      }

      float parentDist = MathUtil::distance2D(p, std::pair<float,float>(parentseg.getX1(),parentseg.getY1()));
      float childDist = MathUtil::distance2D(p, std::pair<float,float>(child.getX0(),child.getY0()));

      if (max(parentDist,childDist) > parentseg.getLength()) {
	// cout << "intersection too far" << endl;
	continue;
      }

      // everything's OK, this child is a reasonable successor.
      parentseg.children.push_back(&child);
    }
  }

  //================================================================
  // Step seven: Search all connected segments to see if any form a loop of length 4.
  // Add those to the quads list.
  vector<Quad> quads;
  
  vector<Segment*> tmp(5);
  for (unsigned int i = 0; i < segments.size(); i++) {
    tmp[0] = &segments[i];
    Quad::search(fimOrig, tmp, segments[i], 0, quads, opticalCenter);
  }

#ifdef DEBUG_APRIL
  {
    for (unsigned int qi = 0; qi < quads.size(); qi++ ) {
      Quad &quad = quads[qi];
      std::pair<float, float> p1 = quad.quadPoints[0];
      std::pair<float, float> p2 = quad.quadPoints[1];
      std::pair<float, float> p3 = quad.quadPoints[2];
      std::pair<float, float> p4 = quad.quadPoints[3];
      cv::line(image, cv::Point2f(p1.first, p1.second), cv::Point2f(p2.first, p2.second), cv::Scalar(0,0,255,0) );
      cv::line(image, cv::Point2f(p2.first, p2.second), cv::Point2f(p3.first, p3.second), cv::Scalar(0,0,255,0) );
      cv::line(image, cv::Point2f(p3.first, p3.second), cv::Point2f(p4.first, p4.second), cv::Scalar(0,0,255,0) );
      cv::line(image, cv::Point2f(p4.first, p4.second), cv::Point2f(p1.first, p1.second), cv::Scalar(0,0,255,0) );

      p1 = quad.interpolate(-1,-1);
      p2 = quad.interpolate(-1,1);
      p3 = quad.interpolate(1,1);
      p4 = quad.interpolate(1,-1);
      cv::circle(image, cv::Point2f(p1.first, p1.second), 3, cv::Scalar(0,255,0,0), 1);
      cv::circle(image, cv::Point2f(p2.first, p2.second), 3, cv::Scalar(0,255,0,0), 1);
      cv::circle(image, cv::Point2f(p3.first, p3.second), 3, cv::Scalar(0,255,0,0), 1);
      cv::circle(image, cv::Point2f(p4.first, p4.second), 3, cv::Scalar(0,255,0,0), 1);
    }
    cv::imshow("debug_april", image);
  }
#endif

  //================================================================
  // Step eight. Decode the quads. For each quad, we first estimate a
  // threshold color to decide between 0 and 1. Then, we read off the
  // bits and see if they make sense.

  std::vector<TagDetection> detections;

  for (unsigned int qi = 0; qi < quads.size(); qi++ ) {
    Quad &quad = quads[qi];

    // Find a threshold
    GrayModel blackModel, whiteModel;
    const int dd = 2 * thisTagFamily.blackBorder + thisTagFamily.dimension;

    for (int iy = -1; iy <= dd; iy++) {
      float y = (iy + 0.5f) / dd;
      for (int ix = -1; ix <= dd; ix++) {
	float x = (ix + 0.5f) / dd;
	std::pair<float,float> pxy = quad.interpolate01(x, y);
	int irx = (int) (pxy.first + 0.5);
	int iry = (int) (pxy.second + 0.5);
	if (irx < 0 || irx >= width || iry < 0 || iry >= height)
	  continue;
	float v = fim.get(irx, iry);
	if (iy == -1 || iy == dd || ix == -1 || ix == dd)
	  whiteModel.addObservation(x, y, v);
	else if (iy == 0 || iy == (dd-1) || ix == 0 || ix == (dd-1))
	  blackModel.addObservation(x, y, v);
      }
    }

    bool bad = false;
    unsigned long long tagCode = 0;
    for ( int iy = thisTagFamily.dimension-1; iy >= 0; iy-- ) {
      float y = (thisTagFamily.blackBorder + iy + 0.5f) / dd;
      for (int ix = 0; ix < thisTagFamily.dimension; ix++ ) {
	float x = (thisTagFamily.blackBorder + ix + 0.5f) / dd;
	std::pair<float,float> pxy = quad.interpolate01(x, y);
	int irx = (int) (pxy.first + 0.5);
	int iry = (int) (pxy.second + 0.5);
	if (irx < 0 || irx >= width || iry < 0 || iry >= height) {
	  // cout << "*** bad:  irx=" << irx << "  iry=" << iry << endl;
	  bad = true;
	  continue;
	}
	float threshold = (blackModel.interpolate(x,y) + whiteModel.interpolate(x,y)) * 0.5f;
	float v = fim.get(irx, iry);
	tagCode = tagCode << 1;
	if ( v > threshold)
	  tagCode |= 1;
#ifdef DEBUG_APRIL
        {
          if (v>threshold)
            cv::circle(image, cv::Point2f(irx, iry), 1, cv::Scalar(0,0,255,0), 2);
          else
            cv::circle(image, cv::Point2f(irx, iry), 1, cv::Scalar(0,255,0,0), 2);
        }
#endif
      }
    }

    if ( !bad ) {
      TagDetection thisTagDetection;
      thisTagFamily.decode(thisTagDetection, tagCode);

      // compute the homography (and rotate it appropriately)
      thisTagDetection.homography = quad.homography.getH();
      thisTagDetection.hxy = quad.homography.getCXY();

      float c = std::cos(thisTagDetection.rotation*(float)M_PI/2);
      float s = std::sin(thisTagDetection.rotation*(float)M_PI/2);
      Eigen::Matrix3d R;
      R.setZero();
      R(0,0) = R(1,1) = c;
      R(0,1) = -s;
      R(1,0) = s;
      R(2,2) = 1;
      Eigen::Matrix3d tmp;
      tmp = thisTagDetection.homography * R;
      thisTagDetection.homography = tmp;

      // Rotate points in detection according to decoded
      // orientation.  Thus the order of the points in the
      // detection object can be used to determine the
      // orientation of the target.
      std::pair<float,float> bottomLeft = thisTagDetection.interpolate(-1,-1);
      int bestRot = -1;
      float bestDist = FLT_MAX;
      for ( int i=0; i<4; i++ ) {
	float const dist = AprilTags::MathUtil::distance2D(bottomLeft, quad.quadPoints[i]);
	if ( dist < bestDist ) {
	  bestDist = dist;
	  bestRot = i;
	}
      }

      for (int i=0; i< 4; i++)
	thisTagDetection.p[i] = quad.quadPoints[(i+bestRot) % 4];

      if (thisTagDetection.good) {
	thisTagDetection.cxy = quad.interpolate01(0.5f, 0.5f);
	thisTagDetection.observedPerimeter = quad.observedPerimeter;
	detections.push_back(thisTagDetection);
      }
    }
  }

#ifdef DEBUG_APRIL
  {
    cv::imshow("debug_april", image);
  }
#endif

  //================================================================
  //Step nine: Some quads may be detected more than once, due to
  //partial occlusion and our aggressive attempts to recover from
  //broken lines. When two quads (with the same id) overlap, we will
  //keep the one with the lowest error, and if the error is the same,
  //the one with the greatest observed perimeter.

  std::vector<TagDetection> goodDetections;

  // NOTE: allow multiple non-overlapping detections of the same target.

  for ( vector<TagDetection>::const_iterator it = detections.begin();
	it != detections.end(); it++ ) {
    const TagDetection &thisTagDetection = *it;

    bool newFeature = true;

    for ( unsigned int odidx = 0; odidx < goodDetections.size(); odidx++) {
      TagDetection &otherTagDetection = goodDetections[odidx];

      if ( thisTagDetection.id != otherTagDetection.id ||
	   ! thisTagDetection.overlapsTooMuch(otherTagDetection) )
	continue;

      // There's a conflict.  We must pick one to keep.
      newFeature = false;

      // This detection is worse than the previous one... just don't use it.
      if ( thisTagDetection.hammingDistance > otherTagDetection.hammingDistance )
	continue;

      // Otherwise, keep the new one if it either has strictly *lower* error, or greater perimeter.
      if ( thisTagDetection.hammingDistance < otherTagDetection.hammingDistance ||
	   thisTagDetection.observedPerimeter > otherTagDetection.observedPerimeter )
	goodDetections[odidx] = thisTagDetection;
    }

     if ( newFeature )
       goodDetections.push_back(thisTagDetection);

  }

  //cout << "AprilTags: edges=" << nEdges << " clusters=" << clusters.size() << " segments=" << segments.size()
  //     << " quads=" << quads.size() << " detections=" << detections.size() << " unique tags=" << goodDetections.size() << endl;

  return goodDetections;
}

} // namespace
