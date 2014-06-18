#ifndef ASLAM_CAMERAS_EQUI_DISTORTION_HPP
#define ASLAM_CAMERAS_EQUI_DISTORTION_HPP

#include <Eigen/Dense>
#include <boost/serialization/nvp.hpp>
#include "StaticAssert.hpp"
#include <sm/PropertyTree.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/boost/serialization.hpp>

namespace aslam {
namespace cameras {

/**
 * \class EquidistantDistortion
 * \brief An implementation of the equidistant distortion model for pinhole cameras.
 *        
 * See "A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses" by Juho Kannala and Sami S. Brandt for further information
 *
 *
 * The usual model of a pinhole camera follows these steps:
 *   - Transformation: Transform the point into a coordinate frame associated with the camera
 *   - Normalization:  Project the point onto the normalized image plane: \f$\mathbf y := \left[ x/z,y/z\right] \f$
 *   - Distortion:     apply a nonlinear transformation to \f$y\f$ to account for distortions caused by the optics
 *   - Projection:     Project the point into the image using a standard \f$3 \time 3\f$ projection matrix
 *
 * This class represents a standard implementation of the distortion block. The function "distort" applies this nonlinear transformation.
 * The function "undistort" applies the inverse transformation. Note that the inverse transformation in this case is not avaialable in 
 * closed form and so it is computed iteratively.
 * 
 */
class EquidistantDistortion {
 public:

  enum {
    IntrinsicsDimension = 4
  };
  enum {
    DesignVariableDimension = IntrinsicsDimension
  };

  /// \brief The default constructor sets all values to zero. 
  EquidistantDistortion();

  /// \brief A constructor that initializes all values.
  EquidistantDistortion(double k1, double k2, double k3, double k4);

  /// \brief initialize from a property tree
  EquidistantDistortion(const sm::PropertyTree & config);

  virtual ~EquidistantDistortion();

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
  double k3() {
    return _k3;
  }
  /// \brief the second tangential distortion parameter
  double k4() {
    return _k4;
  }

  /// \brief Compatibility with boost::serialization.
  enum {
    CLASS_SERIALIZATION_VERSION = 0
  };BOOST_SERIALIZATION_SPLIT_MEMBER();

  template<class Archive>
  void load(Archive & ar, const unsigned int version);

  template<class Archive>
  void save(Archive & ar, const unsigned int version) const;

  bool isBinaryEqual(const EquidistantDistortion & rhs) const;

  static EquidistantDistortion getTestDistortion();

  void clear();

  /// \brief the first distortion parameter
  double _k1;
  /// \brief the second distortion parameter
  double _k2;
  /// \brief the third distortion parameter
  double _k3;
  /// \brief the forth distortion parameter
  double _k4;

};

}  // namespace cameras
}  // namespace aslam

#include "implementation/EquidistantDistortion.hpp"

SM_BOOST_CLASS_VERSION (aslam::cameras::EquidistantDistortion);

#endif /* ASLAM_CAMERAS_EQUI_DISTORTION_HPP */
