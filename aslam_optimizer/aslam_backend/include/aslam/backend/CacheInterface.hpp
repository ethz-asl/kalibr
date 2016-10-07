/*
 * CacheInterface.hpp
 *
 *  Created on: 08.03.2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_ASLAM_BACKEND_CACHEINTERFACE_HPP_
#define INCLUDE_ASLAM_BACKEND_CACHEINTERFACE_HPP_

namespace aslam {
namespace backend {

/**
 * @class CacheInterface
 * @brief Interface for caching expressions
 */
class CacheInterface {
 public:
  /// \brief Constructor
  CacheInterface() : _isCacheValidV(false), _isCacheValidJ(false) { }
  /// \brief Destructor
  virtual ~CacheInterface() { }
  /// \brief Invalidates the cache, derived classes should update the data
  void invalidate() {
    _isCacheValidV = _isCacheValidJ = false;
  }
 protected:
  mutable bool _isCacheValidV; /// \brief Is the cache for the error valid?
  mutable bool _isCacheValidJ; /// \brief Is the cache for the Jacobian valid?
};

} /* namespace aslam */
} /* namespace backend */

#endif /* INCLUDE_ASLAM_BACKEND_CACHEINTERFACE_HPP_ */
