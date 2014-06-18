#ifndef ASLAM_CAMERAS_NO_MASK_HPP
#define ASLAM_CAMERAS_NO_MASK_HPP

namespace sm {
class PropertyTree;
}  // namespace sm

namespace aslam {
namespace cameras {

class NoMask {
 public:

  enum {
    DesignVariableDimension = 0
  };

  NoMask();
  NoMask(const sm::PropertyTree &);
  ~NoMask();

  template<typename K>
  bool isValid(const K & /* k */) const {
    return true;
  }
  bool isBinaryEqual(const NoMask &) const {
    return true;
  }

  // is the mask set? (i.e. mask data != NULL)
  bool isSet () const { return false; }

  /// \brief Compatibility with boost::serialization.
  template<class Archive>
  void serialize(Archive & /* ar */, const unsigned int /* version */) {
  }

  static NoMask getTestMask() {
    return NoMask();
  }

};

}  // namespace cameras
}  // namespace aslam

#endif /* ASLAM_CAMERAS_NO_MASK_HPP */
