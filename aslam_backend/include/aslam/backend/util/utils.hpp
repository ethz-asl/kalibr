/*
 * utils.hpp
 *
 *  Created on: 24.09.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_ASLAM_BACKEND_UTIL_UTILS_HPP_
#define INCLUDE_ASLAM_BACKEND_UTIL_UTILS_HPP_

#include <utility>      // std::pair
#include <vector>

#include <Eigen/Dense>

#include <aslam/backend/DesignVariable.hpp>

namespace aslam {
namespace backend {
namespace utils {

/** branchless signum method */
template <typename T>
inline int sign(const T& val) {
  return (0.0 < val) - (val < 0.0);
}

/** power-2 */
template <typename T>
inline T sqr(const T& val) {
  return val*val;
}

/**
 * \class DesignVariableState
 * Helper class to store the state of a set of design variables to be able to restore it later
 */
class DesignVariableState : private std::vector< std::pair<DesignVariable*, Eigen::MatrixXd> >
{
 private:
  typedef std::pair<DesignVariable*, Eigen::MatrixXd> item_t;
  typedef std::vector<item_t> parent_t;
 public:
  DesignVariableState() { }
  template <typename Iterator>
  DesignVariableState(Iterator begin, Iterator end)
  {
    save(begin, end);
  }
  template <typename Container>
  DesignVariableState(const Container& dvs)
      : DesignVariableState::DesignVariableState(dvs.begin(), dvs.end())
  {
  }

  template <typename Iterator>
  void save(Iterator begin, Iterator end)
  {
    for (Iterator it = begin; it != end; ++it)
      save(*it);
  }
  template <typename Container>
  void save(const Container& dvs)
  {
      save(dvs.begin(), dvs.end());
  }
  void save(DesignVariable* dv)
  {
    Eigen::MatrixXd p;
    dv->getParameters(p);
    this->emplace_back( dv, p );
  }
  void restore() const
  {
    for (auto& dvP : *this)
      dvP.first->setParameters(dvP.second);
  }
  using parent_t::reserve;
  using parent_t::clear;
};

/// \brief Get a list of design variables as vector
template <typename Container>
void getFlattenedDesignVariableParameters(const Container& designVariables, Eigen::VectorXd& v)
{
  // Allocate memory so vector-space design variables fit into the output vector
  int numParametersMin = 0;
  for (auto& dv : designVariables)
    numParametersMin += dv->minimalDimensions();

  v.resize(numParametersMin);

  int cnt = 0;
  Eigen::MatrixXd p;

  for (auto& dv : designVariables) {
    dv->getParameters(p);
    int d = p.size();

    // Resize has to be performed if we have non-vector-space design variables
    const int newSize = cnt + d;
    if (newSize > numParametersMin)
      v.conservativeResize(newSize);

    v.segment(cnt, d) = Eigen::Map<Eigen::VectorXd>(p.data(), d, 1);
    cnt += d;
  }
}

/// \brief Get a list of design variables as vector
template <typename Container>
Eigen::VectorXd getFlattenedDesignVariableParameters(const Container& designVariables)
{
  Eigen::VectorXd p;
  getFlattenedDesignVariableParameters(designVariables, p);
  return p;
}

template <typename Container, typename Vector>
void applyStateUpdate(const Container& designVariables, const Vector& dx)
{
  static_assert(Vector::RowsAtCompileTime == 1 || Vector::ColsAtCompileTime == 1, "");

  // Apply the update to the dense state.
  int startIdx = 0;
  for (auto dv : designVariables) {
    const int dbd = dv->minimalDimensions();
    Eigen::VectorXd dxS = dx.segment(startIdx, dbd);
    dxS *= dv->scaling();
    dv->update(&dxS[0], dbd);
    startIdx += dbd;
  }
}

}
}
}

#endif /* INCLUDE_ASLAM_BACKEND_UTIL_UTILS_HPP_ */
