#ifndef ASLAM_CAMERAS_FOV_DISTORTION_HPP
#define ASLAM_CAMERAS_FOV_DISTORTION_HPP

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
 * \class FovDistortion
 * \brief An implementation of the fov distortion model for pinhole cameras.
 */
class FovDistortion {
 public:

  enum {
    IntrinsicsDimension = 1
  };
  enum {
    DesignVariableDimension = IntrinsicsDimension
  };

  /// \brief The default constructor sets all values to zero. 
  FovDistortion();

  /// \brief A constructor that initializes all values.
  FovDistortion(double w);

  /// \brief initialize from a property tree
  FovDistortion(const sm::PropertyTree & config);

  virtual ~FovDistortion();

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

  double w() {
    return _w;
  }

  /// Static function that checks whether the given intrinsic parameters are valid for this model.
  bool areParametersValid(double w) {
    return std::abs(w) < 1e-16 || (w >= kMinValidW && w <= kMaxValidW);
  }

  /// \brief Compatibility with boost::serialization.
  enum {
    CLASS_SERIALIZATION_VERSION = 0
  };BOOST_SERIALIZATION_SPLIT_MEMBER();

  template<class Archive>
  void load(Archive & ar, const unsigned int version);

  template<class Archive>
  void save(Archive & ar, const unsigned int version) const;

  bool isBinaryEqual(const FovDistortion & rhs) const;

  static FovDistortion getTestDistortion();

  void clear();

  /// \brief the disortion parameter
  double _w;
  static constexpr double kMaxValidAngle = (89.0 * M_PI / 180.0);
  static constexpr double kMinValidW = 0.5;
  static constexpr double kMaxValidW = 1.5;
};

}  // namespace cameras
}  // namespace aslam

#include "implementation/FovDistortion.hpp"

SM_BOOST_CLASS_VERSION (aslam::cameras::FovDistortion);

#endif /* ASLAM_CAMERAS_FOV_DISTORTION_HPP */
