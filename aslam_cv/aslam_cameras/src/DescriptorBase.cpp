#include <aslam/DescriptorBase.hpp>

namespace aslam {

DescriptorBase::DescriptorBase() {

}
DescriptorBase::~DescriptorBase() {

}

}  // namespace aslam

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
// These ones are in sm_boost
#include <boost/portable_binary_iarchive.hpp>
#include <boost/portable_binary_oarchive.hpp>

BOOST_CLASS_EXPORT_IMPLEMENT (aslam::DescriptorBase);

