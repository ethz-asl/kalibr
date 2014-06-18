/**
 * @file   Descriptor.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>
 * @date   Sat Nov 24 11:30:38 2012
 * 
 * @brief  A base class for descriptors
 * 
 * 
 */

#ifndef ASLAM_DESCRIPTOR_HPP
#define ASLAM_DESCRIPTOR_HPP

#include <sm/boost/serialization.hpp>
#include <boost/serialization/export.hpp>
#include <boost/shared_ptr.hpp>

namespace aslam {
class DescriptorBase;

SM_DEFINE_EXCEPTION(DescriptorCastException, std::runtime_error);

template<typename D>
D * descriptor_cast(DescriptorBase * descriptor) {
#ifndef NDEBUG
  D * sd = dynamic_cast<D *>(descriptor);
  SM_ASSERT_TRUE(
      DescriptorCastException,
      sd != NULL,
      "Tried to compare two descriptors of incomparible types. This type is "
          << typeid(D).name() << ", the other type is "
          << typeid(descriptor).name() << ".");
#else
  D * sd = static_cast< D * >(descriptor);
#endif
  return sd;
}

template<typename D>
const D * descriptor_cast(const DescriptorBase * descriptor) {
#ifndef NDEBUG
  const D * sd = dynamic_cast<const D *>(descriptor);
  SM_ASSERT_TRUE(
      DescriptorCastException,
      sd != NULL,
      "Tried to compare two descriptors of incomparible types. This type is "
          << typeid(D).name() << ", the other type is "
          << typeid(descriptor).name() << ".");
#else
  const D * sd = static_cast< const D * >(descriptor);
#endif
  return sd;
}

/// \brief A  class for descriptors. This doesn't have an interface.
class DescriptorBase {
 public:
  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

  typedef float distance_t;
  typedef boost::shared_ptr<DescriptorBase> Ptr;

  DescriptorBase();
  virtual ~DescriptorBase();

  /// \brief compute the distance between two descriptors
  virtual float distance(const DescriptorBase & other) const = 0;

  /// \brief initialized from raw data (such as a cv::Mat)
  virtual void initFromArray(const unsigned char * descriptionVector) = 0;

  /// \brief clone the underlying descriptor
  virtual DescriptorBase * clone() const = 0;

  /// \brief are the two descriptors binary equal?
  virtual bool isBinaryEqual(const DescriptorBase & rhs) const = 0;

  /// \brief support for unit testing.
  virtual void setRandom() = 0;

  template<typename DERIVED_T>
  DERIVED_T & as() {
    return *descriptor_cast<DERIVED_T>(this);
  }

  template<typename DERIVED_T>
  const DERIVED_T & as() const {
    return *descriptor_cast<DERIVED_T>(this);
  }

  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
  enum {
    CLASS_SERIALIZATION_VERSION = 1
  };BOOST_SERIALIZATION_SPLIT_MEMBER();

  template<class Archive>
  void load(Archive & /* ar */, const unsigned int /* version */) {
  }

  template<class Archive>
  void save(Archive & /* ar */, const unsigned int /* version */) const {
  }

};

}  // namespace aslam

SM_BOOST_CLASS_VERSION (aslam::DescriptorBase);

#endif /* ASLAM_DESCRIPTOR_HPP */
