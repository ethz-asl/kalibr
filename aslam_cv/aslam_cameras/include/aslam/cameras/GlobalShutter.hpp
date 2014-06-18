#ifndef ASLAM_GLOBAL_SHUTTER_HPP
#define ASLAM_GLOBAL_SHUTTER_HPP

#include <aslam/Time.hpp>
#include <Eigen/Core>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/boost/serialization.hpp>

namespace sm {
class PropertyTree;
}  // namespace sm

namespace aslam {
namespace cameras {

class GlobalShutter {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum {
    DesignVariableDimension = 0
  };

  GlobalShutter();
  GlobalShutter(const sm::PropertyTree & config);
  ~GlobalShutter();

  template<typename K>
  Duration temporalOffset(const K & /* k */ ) const {
    return Duration(0);
  }

  template<typename DERIVED_K, typename DERIVED_J>
  void temporalOffsetIntrinsicsJacobian(
      const Eigen::MatrixBase<DERIVED_K> & /* k */,
      const Eigen::MatrixBase<DERIVED_J> & outJ) const {
    Eigen::MatrixBase<DERIVED_J> & J =
        const_cast<Eigen::MatrixBase<DERIVED_J> &>(outJ);
    J.resize(0, 0);

  }

  // aslam::backend compatibility
  void update(const double * v);
  int minimalDimensions() const;
  void getParameters(Eigen::MatrixXd & P) const;
  void setParameters(const Eigen::MatrixXd & P);
  Eigen::Vector2i parameterSize() const;

  /// \brief Compatibility with boost::serialization.
  enum {
    CLASS_SERIALIZATION_VERSION = 0
  };BOOST_SERIALIZATION_SPLIT_MEMBER();
  template<class Archive>
  void load(Archive & /* ar */, const unsigned int /* version */) {
  }
  template<class Archive>
  void save(Archive & /* ar */, const unsigned int /* version */) const {
  }

  bool isBinaryEqual(const GlobalShutter & /* rhs */) const {
    return true;
  }

  static GlobalShutter getTestShutter() {
    return GlobalShutter();
  }
};

}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION (aslam::cameras::GlobalShutter);

#endif /* ASLAM_GLOBAL_SHUTTER_HPP */
