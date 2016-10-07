/*
 * CacheExpression.hpp
 *
 *  Created on: 08.03.2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_ASLAM_BACKEND_CACHEEXPRESSION_HPP_
#define INCLUDE_ASLAM_BACKEND_CACHEEXPRESSION_HPP_

// boost includes
#include <boost/thread.hpp>

// Eigen includes
#include <Eigen/Dense>

// aslam_backend includes
#include <aslam/Exceptions.hpp>

// self includes
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/JacobianContainerSparse.hpp>
#include <aslam/backend/JacobianContainerPrescale.hpp>
#include <aslam/backend/CacheInterface.hpp>

namespace aslam {
namespace backend {

template<int IRows, int ICols, typename TScalar>
class GenericMatrixExpressionNode;

/**
 * \class CacheExpressionNode
 * \brief Wraps an expression into a cache data structure to avoid duplicate
 * computation of error and Jacobian values
 *
 * \tparam ExpressionNode Type of the expression node
 * \tparam Dimensions Dimensionality of the design variables
 */
template <typename ExpressionNode, int Dimension>
class CacheExpressionNode : public CacheInterface, public ExpressionNode
{
 public:
  template <typename Expression>
  friend Expression toCacheExpression(const Expression& expr);

 public:
  virtual ~CacheExpressionNode() { }

 protected:

  typename ExpressionNode::value_t evaluateImplementation() const
  {
    if (!_isCacheValidV)
    {
      boost::mutex::scoped_lock lock(_mutexV);
      if (!_isCacheValidV) // could be updated by another thread in the meantime
      {
        _v = _node->evaluate();
        _isCacheValidV = true;
      }
    }
    return _v;
  }

  void evaluateJacobiansImplementation(JacobianContainer & outJacobians) const override
  {
    updateJacobian();
    _jc.addTo(outJacobians);
  }

  virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const override
  {
    _node->getDesignVariables(designVariables);
  }

 private:

  CacheExpressionNode(const boost::shared_ptr<ExpressionNode>& e)
      : CacheInterface(), ExpressionNode(), _node(e)
  {

  }

  void updateJacobian() const
  {
    if (!_isCacheValidJ)
    {
      boost::mutex::scoped_lock lock(_mutexJ);
      if (!_isCacheValidJ) // could be updated by another thread in the meantime
      {
        _jc.setZero();
        _node->evaluateJacobians(_jc);
        _isCacheValidJ = true;
      }
    }
  }

 private:
  mutable typename ExpressionNode::value_t _v; /// \brief Cache for error values
  mutable JacobianContainerSparse<Dimension> _jc = JacobianContainerSparse<Dimension>(Dimension); /// \brief Cache for Jacobians
  boost::shared_ptr<ExpressionNode> _node; /// \brief Wrapped expression node, stored to delegate evaluation calls
  mutable boost::mutex _mutexV; /// \brief Mutex for error value write operations
  mutable boost::mutex _mutexJ; /// \brief Mutex for Jacobian write operations
};



template<int IRows, int ICols, int Dimension, typename TScalar>
class CacheExpressionNode< GenericMatrixExpressionNode<IRows, ICols, TScalar>, Dimension > : public CacheInterface, public GenericMatrixExpressionNode<IRows, ICols, TScalar>
{
 public:
  template <typename Expression>
  friend Expression toCacheExpression(const Expression& expr);
  typedef GenericMatrixExpressionNode<IRows, ICols, TScalar> ExpressionNode;

 public:
  virtual ~CacheExpressionNode() { }

 protected:

  void evaluateImplementation() const
  {
    if (!_isCacheValidV)
    {
      boost::mutex::scoped_lock lock(_mutexV);
      if (!_isCacheValidV) // could be updated by another thread in the meantime
      {
        this->_currentValue = _node->evaluate();
        _isCacheValidV = true;
      }
    }
  }

  void evaluateJacobiansImplementation(JacobianContainer & outJacobians, const typename ExpressionNode::differential_t & chainRuleDifferential) const override
  {
    updateJacobian();
    _jc.addTo((JacobianContainer&)applyDifferentialToJacobianContainer(outJacobians, chainRuleDifferential, IRows));
  }

  virtual void getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const override
  {
    _node->getDesignVariables(designVariables);
  }

 private:

  CacheExpressionNode(const boost::shared_ptr<ExpressionNode>& e)
      : CacheInterface(), ExpressionNode(), _node(e)
  {

  }

  void updateJacobian() const
  {
    if (!_isCacheValidJ)
    {
      boost::mutex::scoped_lock lock(_mutexJ);
      if (!_isCacheValidJ) // could be updated by another thread in the meantime
      {
        _jc.setZero();
        _node->evaluateJacobians(_jc, IdentityDifferential<typename ExpressionNode::tangent_vector_t, TScalar>());
        _isCacheValidJ = true;
      }
    }
  }

 private:
  mutable JacobianContainerSparse<IRows> _jc = JacobianContainerSparse<IRows>(IRows); /// \brief Cache for Jacobians
  boost::shared_ptr<ExpressionNode> _node; /// \brief Wrapped expression node, stored to delegate evaluation calls
  mutable boost::mutex _mutexV; /// \brief Mutex for error value write operations
  mutable boost::mutex _mutexJ; /// \brief Mutex for Jacobian write operations
};


/**
 * \brief Converts a regular expression to a cache expression and registers the expression
 * in the corresponding design variables in order to allow the design variables to
 * invalidate the cache
 *
 * @param expr original expression
 * \tparam Expression expression type
 * @return Cached expression
 */
template <typename Expression>
Expression toCacheExpression(const Expression& expr)
{
  boost::shared_ptr< CacheExpressionNode<typename Expression::node_t, Expression::Dimension> > node
      (new CacheExpressionNode<typename Expression::node_t, Expression::Dimension>(expr.root()));
  DesignVariable::set_t dvs;
  node->getDesignVariables(dvs);
  for (auto dv : dvs)
    dv->registerCacheExpressionNode(node);
  return Expression(node);
}


} /* namespace aslam */
} /* namespace backend */


#endif /* INCLUDE_ASLAM_BACKEND_CACHEEXPRESSION_HPP_ */
