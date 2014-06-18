#ifndef TAGDETECTION_H
#define TAGDETECTION_H

#include <Eigen/Dense>

#include "opencv2/opencv.hpp"

#include <utility>
#include <vector>

namespace AprilTags {

struct TagDetection {

  //! Constructor
  TagDetection();

  //! Constructor for manually creating tags in a world map
  TagDetection(int id);

  //! Is the detection good enough?
  bool good;

  //! Observed code
  long long obsCode;

  //! Matched code
  long long code;

  //! What was the ID of the detected tag?
  int id;

  //! The hamming distance between the detected code and the true code
  int hammingDistance;
  
  //! How many 90 degree rotations were required to align the code (internal use only)
  int rotation;

  /////////////// Fields below are filled in by TagDetector ///////////////
  //! Position (in fractional pixel coordinates) of the detection.
  /*  The points travel counter-clockwise around the target, always
   *  starting from the same corner of the tag.
   */
  std::pair<float,float> p[4];

  //! Center of tag in pixel coordinates.
  std::pair<float,float> cxy;

  //! Measured in pixels, how long was the observed perimeter.
  /*! Observed perimeter excludes the inferred perimeter which is used to connect incomplete quads. */
  float observedPerimeter;

  //! A 3x3 homography that computes pixel coordinates from tag-relative coordinates.
  /*  Both the input and output coordinates are 2D homogeneous vectors, with y = Hx.
   *  'y' are pixel coordinates, 'x' are tag-relative coordinates. Tag coordinates span
   *  from (-1,-1) to (1,1). The orientation of the homography reflects the orientation
   *  of the target.
   */
  Eigen::Matrix3d homography;

  //! Orientation in the xy plane
  float getXYOrientation() const;

  //! The homography is relative to image center, whose coordinates are below.
  std::pair<float,float> hxy;

  //! Interpolate point given (x,y) is in tag coordinate space from (-1,-1) to (1,1).
  std::pair<float,float> interpolate(float x, float y) const;

  //! Used to eliminate redundant tags
  bool overlapsTooMuch(const TagDetection &other) const;

  //! Relative pose of tag with respect to the camera
  /* Returns the relative location and orientation of the tag using a
     4x4 homogeneous transformation matrix (see Hartley&Zisserman,
     Multi-View Geometry, 2003). Requires knowledge of physical tag
     size (side length of black square in meters) as well as camera
     calibration (focal length and principal point); Result is in
     camera frame (z forward, x right, y down)
  */
  Eigen::Matrix4d getRelativeTransform(double tag_size, double fx, double fy,
                                       double px, double py) const;

  //! Recover rotation matrix and translation vector of April tag relative to camera.
  // Result is in object frame (x forward, y left, z up)
  void getRelativeTranslationRotation(double tag_size, double fx, double fy, double px, double py,
                                      Eigen::Vector3d& trans, Eigen::Matrix3d& rot) const;

  //! Draw the detection within the supplied image, including boarders and tag ID.
  void draw(cv::Mat& image) const;

  //! Compare function to sort detections by std::sort
  static bool sortByIdCompare (const TagDetection &a, const TagDetection &b) { return (a.id<b.id); }

};

} // namespace

#endif
