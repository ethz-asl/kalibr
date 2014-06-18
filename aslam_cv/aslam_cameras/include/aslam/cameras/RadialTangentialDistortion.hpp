#ifndef ASLAM_CAMERAS_RT_DISTORTION_HPP
#define ASLAM_CAMERAS_RT_DISTORTION_HPP

#include <Eigen/Dense>
#include <boost/serialization/nvp.hpp>
#include "StaticAssert.hpp"
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/boost/serialization.hpp>

namespace sm {
class PropertyTree;
}  // namespace sm

namespace aslam {
namespace cameras {

/**
 * \class RadialTangentialDistortion
 * \brief An implementation of the standard, four parameter distortion model for pinhole cameras.
 *        
 * \todo outline the math here and provide a reference. What is the original reference?
 *
 *
 * The usual model of a pinhole camera follows these steps:
 *   - Transformation: Transform the point into a coordinate frame associated with the camera
 *   - Normalization:  Project the point onto the normalized image plane: \f$\mathbf y := \left[ x/z,y/z\right] \f$
 *   - Distortion:     apply a nonlinear transformation to \f$y\f$ to account for radial and tangential distortion of the lens
 *   - Projection:     Project the point into the image using a standard \f$3 \time 3\f$ projection matrix
 *
 * This class represents a standard implementation of the distortion block. The function "distort" applies this nonlinear transformation.
 * The function "undistort" applies the inverse transformation. Note that the inverse transformation in this case is not avaialable in 
 * closed form and so it is computed iteratively.
 * 
 */
class RadialTangentialDistortion {
 public:

  enum {
    IntrinsicsDimension = 4
  };
  enum {
    DesignVariableDimension = IntrinsicsDimension
  };

  /// \brief The default constructor sets all values to zero. 
  RadialTangentialDistortion();

  /// \brief A constructor that initializes all values.
  RadialTangentialDistortion(double k1, double k2, double p1, double p2);

  RadialTangentialDistortion(const sm::PropertyTree & config);

  virtual ~RadialTangentialDistortion();

  /** 
   * \brief Apply distortion to a point in the normalized image plane
   * 
   * @param y The point in the normalized image plane. After the function, this point is distorted.
   */
  template<typename DERIVED_Y>
  void distort(const Eigen::MatrixBase<DERIVED_Y> & y) const;

  /** 
   * 
   * \brief Apply distortion to a point in the normalized image plane
   * 
   * @param y The point in the normalized image plane. After the function, this point is distorted.
   * @param outJy The Jacobian of the distortion function with respect to small changes in the input point.
   */
  template<typename DERIVED_Y, typename DERIVED_JY>
  void distort(const Eigen::MatrixBase<DERIVED_Y> & y,
               const Eigen::MatrixBase<DERIVED_JY> & outJy) const;

  /** 
   * \brief Apply undistortion to recover a point in the normalized image plane.
   * 
   * @param y The distorted point. After the function, this point is in the normalized image plane.
   */
  template<typename DERIVED>
  void undistort(const Eigen::MatrixBase<DERIVED> & y) const;

  /** 
   * \brief Apply undistortion to recover a point in the normalized image plane.
   * 
   * @param y The distorted point. After the function, this point is in the normalized image plane.
   * @param outJy The Jacobian of the undistortion function with respect to small changes in the input point.
   */
  template<typename DERIVED, typename DERIVED_JY>
  void undistort(const Eigen::MatrixBase<DERIVED> & y,
                 const Eigen::MatrixBase<DERIVED_JY> & outJy) const;

  /** 
   * \brief Apply distortion to the point and provide the Jacobian of the distortion with respect to small changes in the distortion parameters
   * 
   * @param imageY the point in the normalized image plane.
   * @param outJd  the Jacobian of the distortion with respect to small changes in the distortion parameters.
   */
  template<typename DERIVED_Y, typename DERIVED_JD>
  void distortParameterJacobian(
      const Eigen::MatrixBase<DERIVED_Y> & imageY,
      const Eigen::MatrixBase<DERIVED_JD> & outJd) const;

  /** 
   * \brief A function for compatibility with the aslam backend. This implements an update of the distortion parameter.
   * 
   * @param v A double array representing the update vector.
   */
  void update(const double * v);

  /** 
   * \brief A function for compatibility with the aslam backend. 
   * 
   * @param v The number of parameters expected by the update equation. This should also define the number of columns in the matrix returned by distortParameterJacobian.
   */
  int minimalDimensions() const;

  /** 
   * \brief A function for compatibility with the aslam backend. 
   * 
   * @param P This matrix is resized and filled with parameters representing the full state of the distortion. 
   */
  void getParameters(Eigen::MatrixXd & P) const;

  /** 
   * \brief A function for compatibility with the aslam backend. 
   * 
   * @param P The full state of the distortion class is set from the matrix of parameters.
   */
  void setParameters(const Eigen::MatrixXd & P);

  Eigen::Vector2i parameterSize() const;

  /// \brief the first radial distortion parameter
  double k1() {
    return _k1;
  }
  /// \brief the second radial distortion parameter
  double k2() {
    return _k2;
  }
  /// \brief the first tangential distortion parameter
  double p1() {
    return _p1;
  }
  /// \brief the second tangential distortion parameter
  double p2() {
    return _p2;
  }

  void clear() {
    _k1 = 0.0;
    _k2 = 0.0;
    _p1 = 0.0;
    _p2 = 0.0;
  }

  /// \brief Compatibility with boost::serialization.
  enum {
    CLASS_SERIALIZATION_VERSION = 0
  };BOOST_SERIALIZATION_SPLIT_MEMBER();
  template<class Archive>
  void load(Archive & ar, const unsigned int version);
  template<class Archive>
  void save(Archive & ar, const unsigned int version) const;

  bool isBinaryEqual(const RadialTangentialDistortion & rhs) const;

  static RadialTangentialDistortion getTestDistortion();

  /// \brief the first radial distortion parameter
  double _k1;
  /// \brief the second radial distortion parameter
  double _k2;
  /// \brief the first tangential distortion parameter
  double _p1;
  /// \brief the second tangential distortion parameter
  double _p2;

};

}  // namespace cameras
}  // namespace aslam

#include "implementation/RadialTangentialDistortion.hpp"

SM_BOOST_CLASS_VERSION (aslam::cameras::RadialTangentialDistortion);

#endif /* ASLAM_CAMERAS_DISTORTION_HPP */
