#ifndef ASLAM_CAMERAS_NO_DISTORTION_HPP
#define ASLAM_CAMERAS_NO_DISTORTION_HPP

#include <Eigen/Core>
#include "StaticAssert.hpp"
namespace sm {
class PropertyTree;
}  // namespace sm

namespace aslam {
namespace cameras {

class NoDistortion {
 public:

  enum {
    DesignVariableDimension = 0
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum {
    IntrinsicsDimension = -1  //!< The number of distortion coefficients
  };

  NoDistortion();
  NoDistortion(const sm::PropertyTree & config);
  virtual ~NoDistortion();

  template<typename DERIVED_Y>
  void distort(const Eigen::MatrixBase<DERIVED_Y> & y) const;

  template<typename DERIVED_Y, typename DERIVED_JY>
  void distort(const Eigen::MatrixBase<DERIVED_Y> & y,
               const Eigen::MatrixBase<DERIVED_JY> & outJy) const;

  template<typename DERIVED>
  void undistort(const Eigen::MatrixBase<DERIVED> & y) const;

  template<typename DERIVED, typename DERIVED_JY>
  void undistort(const Eigen::MatrixBase<DERIVED> & y,
                 const Eigen::MatrixBase<DERIVED_JY> & outJy) const;

  template<typename DERIVED_Y, typename DERIVED_JD>
  void distortParameterJacobian(
      const Eigen::MatrixBase<DERIVED_Y> & imageY,
      const Eigen::MatrixBase<DERIVED_JD> & outJd) const;

  // aslam::backend compatibility
  void update(const double * v);
  int minimalDimensions() const;
  void getParameters(Eigen::MatrixXd & P) const;
  void setParameters(const Eigen::MatrixXd & P);
  Eigen::Vector2i parameterSize() const;

  template<class Archive>
  void serialize(Archive & /* ar */, const unsigned int /* version */) {
  }

  bool isBinaryEqual(const NoDistortion &) const {
    return true;
  }

  static NoDistortion getTestDistortion() {
    return NoDistortion();
  }

  void clear() {
  }

};

}  // namespace cameras
}  // namespace aslam

#include "implementation/NoDistortion.hpp"

#endif /* ASLAM_CAMERAS_DISTORTION_HPP */
