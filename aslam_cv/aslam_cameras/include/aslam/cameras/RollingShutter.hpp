#ifndef ASLAM_ROLLING_SHUTTER_HPP
#define ASLAM_ROLLING_SHUTTER_HPP

#include <aslam/Time.hpp>
#include <Eigen/Core>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/boost/serialization.hpp>

namespace sm {
class PropertyTree;
}  // namespace sm

namespace aslam {
namespace cameras {

class RollingShutter {

 public:

  enum {
    DesignVariableDimension = 0
  };

  RollingShutter();
  RollingShutter(const sm::PropertyTree & config);
  RollingShutter(double lineDelay);
  ~RollingShutter();

  template<typename DERIVED_K>
  Duration temporalOffset(const Eigen::MatrixBase<DERIVED_K> & k) const {
    return Duration(_lineDelay * k[1]);
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
  void load(Archive & ar, const unsigned int version) {
    SM_ASSERT_LE(std::runtime_error, version,
                 (unsigned int) CLASS_SERIALIZATION_VERSION,
                 "Unsupported serialization version");

    ar >> BOOST_SERIALIZATION_NVP(_lineDelay);
  }
  template<class Archive>
  void save(Archive & ar, const unsigned int /* version */) const {
    ar << BOOST_SERIALIZATION_NVP(_lineDelay);
  }

  bool isBinaryEqual(const RollingShutter & rhs) const;

  static RollingShutter getTestShutter();

  double lineDelay() const {
    return _lineDelay;
  }
  ;

  /// The resulting jacobian assumes the output of "temporal offset" is in seconds.
  template<typename DERIVED_K, typename DERIVED_J>
  void temporalOffsetIntrinsicsJacobian(
      const Eigen::MatrixBase<DERIVED_K> & k,
      const Eigen::MatrixBase<DERIVED_J> & outJ) const {
    Eigen::MatrixBase<DERIVED_J> & J =
        const_cast<Eigen::MatrixBase<DERIVED_J> &>(outJ);
    J.resize(1, 1);
    J(0, 0) = k[1];
  }

 private:
  // RS camera time between start of integration of two consecutive lines in seconds
  double _lineDelay;

};

}  // namespace cameras
}  // namespace aslam

SM_BOOST_CLASS_VERSION (aslam::cameras::RollingShutter);
#endif /* ASLAM_ROLLING_SHUTTER_HPP */
