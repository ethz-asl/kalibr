#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/FullMatcher.hpp>
#include <aslam/DenseMatcher.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <aslam/DescriptorMatchingAlgorithm.hpp>

template<typename INDEX_T>
void exportMatch(const std::string & indexName) {
  using namespace boost::python;
  using namespace aslam;

  typedef Match<INDEX_T> match_t;
  class_ < match_t
      > ((indexName + "Match").c_str(), init<>()).def(
          init<INDEX_T, INDEX_T, float>()).def("item", &match_t::item).def(
          "getItem0", &match_t::getItem0).def("getItem1", &match_t::getItem1)
          .def_readonly("distance", &match_t::distance).def_readwrite(
          "isTriangulated", &match_t::isTriangulated);

}

void exportMatchingAlgorithms() {
  using namespace boost::python;
  using namespace aslam;

  class_<DenseMatcher, boost::shared_ptr<DenseMatcher> >("DenseMatcher",
                                                         init<>()).def(
      init<boost::uint8_t, boost::uint8_t>()).def("match",
                                                  &DenseMatcher::matchSlow);

  exportMatch<KeypointIdentifier>("KeypointIdentifier");

  class_<MatchingAlgorithm, boost::shared_ptr<MatchingAlgorithm>,
      boost::noncopyable>("MatchingAlgorithm", no_init).def(
      "doSetup", &MatchingAlgorithm::doSetup).def("sizeA",
                                                  &MatchingAlgorithm::sizeA).def(
      "sizeB", &MatchingAlgorithm::sizeB)
  // /// distances above this threshold will not be returned as matches.
      .def("distanceThreshold", &MatchingAlgorithm::distanceThreshold)
  // /// \brief Should we skip the item in list A? This will be called once for each item in the list
  // virtual bool skipA(size_t indexA) const = 0;
      .def("skipA", &MatchingAlgorithm::skipA)
  // /// \brief Should we skip the item in list B? This will be called many times.
  // virtual bool skipB(size_t indexB) const = 0;
      .def("skipB", &MatchingAlgorithm::skipB)
  // /// \brief the "distance" between the two points.
  // ///        For points that absolutely don't match. Please use float max.
  // virtual float distance(size_t indexA, size_t indexB) const = 0;
      .def("distance", &MatchingAlgorithm::distance);

  class_<FullMatcher, boost::shared_ptr<FullMatcher> >("FullMatcher", init<>())
      .def("initParameters", &FullMatcher::initParameters).def(
      "setParameters",
      &FullMatcher::setParameters,
      "setParameters(float distanceThreshold, double pureRotationEpipolarThreshold, double reprojectionThreshold, double epipolarThreshold);")
      .def("setMatchData", &FullMatcher::setMatchData);

  class_ < std::vector<kid_match_t>
      > ("KeypointIdentifierMatchVector").def(
          vector_indexing_suite<std::vector<kid_match_t> >());

  class_<KeypointIdentifierMatchingAlgorithm,
      boost::shared_ptr<KeypointIdentifierMatchingAlgorithm>,
      bases<MatchingAlgorithm>, boost::noncopyable>(
      "KeypointIdentifierMatchingAlgorithm", no_init).def(
      "getMatches", &KeypointIdentifierMatchingAlgorithm::getMatches,
      return_value_policy<copy_const_reference>());

  class_<DescriptorMatchingAlgorithm,
      boost::shared_ptr<DescriptorMatchingAlgorithm>,
      bases<KeypointIdentifierMatchingAlgorithm> >(
      "DescriptorMatchingAlgorithm", init<>()).def(
      init<const sm::PropertyTree &>()).def(
      init<MultiFrameId, int, FrameBase *, MultiFrameId, int, FrameBase *, float>())
      .def("initParameters", &DescriptorMatchingAlgorithm::initParameters).def(
      "setParameters", &DescriptorMatchingAlgorithm::setParameters).def(
      "setFrames", &DescriptorMatchingAlgorithm::setFrames);

}
