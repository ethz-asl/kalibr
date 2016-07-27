#include <aslam/backend/EuclideanExpressionNode.hpp>
#include <aslam/backend/VectorExpressionNode.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <aslam/backend/HomogeneousExpressionNode.hpp>
namespace aslam {
  namespace backend {
    
    EuclideanExpressionNode::EuclideanExpressionNode()
    {

    }

    EuclideanExpressionNode::~EuclideanExpressionNode()
    {

    }


    /// \brief Evaluate the euclidean matrix.
    Eigen::Vector3d EuclideanExpressionNode::toEuclidean() const
    {
      return toEuclideanImplementation();
    }

      
    /// \brief Evaluate the Jacobians
    void EuclideanExpressionNode::evaluateJacobians(JacobianContainer & outJacobians) const
    {
      evaluateJacobiansImplementation(outJacobians);
    }
   
    
    /// \brief Evaluate the Jacobians and apply the chain rule.
    void EuclideanExpressionNode::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      SM_ASSERT_EQ_DBG(Exception, applyChainRule.cols(), 3, "The chain rule matrix must have three columns");
      evaluateJacobiansImplementation(outJacobians, applyChainRule);
    }

    void EuclideanExpressionNode::getDesignVariables(DesignVariable::set_t & designVariables) const
    {
      getDesignVariablesImplementation(designVariables);
    }


    EuclideanExpressionNodeMultiply::EuclideanExpressionNodeMultiply(boost::shared_ptr<RotationExpressionNode> lhs, boost::shared_ptr<EuclideanExpressionNode> rhs) :
         _lhs(lhs), _rhs(rhs)
       {

    	_C_lhs = _lhs->toRotationMatrix();
         _p_rhs = _rhs->toEuclidean();
       }

    EuclideanExpressionNodeMultiply::~EuclideanExpressionNodeMultiply()
    {

    }


    void EuclideanExpressionNodeMultiply::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _lhs->getDesignVariables(designVariables);
      _rhs->getDesignVariables(designVariables);
    }


    Eigen::Vector3d EuclideanExpressionNodeMultiply::toEuclideanImplementation() const
    {
      _C_lhs = _lhs->toRotationMatrix();
      _p_rhs = _rhs->toEuclidean();

      return _C_lhs * _p_rhs;
    }

    void EuclideanExpressionNodeMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      _lhs->evaluateJacobians(outJacobians, sm::kinematics::crossMx(_C_lhs * _p_rhs));
      _rhs->evaluateJacobians(outJacobians, _C_lhs);
    }

    void EuclideanExpressionNodeMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      _lhs->evaluateJacobians(outJacobians, applyChainRule * sm::kinematics::crossMx(_C_lhs * _p_rhs));
      _rhs->evaluateJacobians(outJacobians, applyChainRule * _C_lhs);
    }


    // -------------------------------------------------------
    // ## New Class for rotations with MatrixExpressions
    EuclideanExpressionNodeMatrixMultiply::EuclideanExpressionNodeMatrixMultiply(boost::shared_ptr<MatrixExpressionNode> lhs, boost::shared_ptr<EuclideanExpressionNode> rhs) :
         _lhs(lhs), _rhs(rhs)
       {

    	 _A_lhs = _lhs->toMatrix3x3();
         _p_rhs = _rhs->toEuclidean();
       }

    EuclideanExpressionNodeMatrixMultiply::~EuclideanExpressionNodeMatrixMultiply()
    {
      
    }


    void EuclideanExpressionNodeMatrixMultiply::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _lhs->getDesignVariables(designVariables);
      _rhs->getDesignVariables(designVariables);
    }

    
    Eigen::Vector3d EuclideanExpressionNodeMatrixMultiply::toEuclideanImplementation() const
    {
      _A_lhs = _lhs->toMatrix3x3();
      _p_rhs = _rhs->toEuclidean();

      return _A_lhs * _p_rhs;
    }

    void EuclideanExpressionNodeMatrixMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
    	Eigen::Matrix<double, 3,9> J_full;
    	J_full << _p_rhs(0) * Eigen::Matrix3d::Identity(), _p_rhs(1) * Eigen::Matrix3d::Identity(), _p_rhs(2) * Eigen::Matrix3d::Identity();
    	_lhs->evaluateJacobians(outJacobians, J_full);
        _rhs->evaluateJacobians(outJacobians, _A_lhs);
    }

    void EuclideanExpressionNodeMatrixMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
    	Eigen::Matrix<double, 3,9> J_full;
    	J_full << _p_rhs(0) * Eigen::Matrix3d::Identity(), _p_rhs(1) * Eigen::Matrix3d::Identity(), _p_rhs(2) * Eigen::Matrix3d::Identity();
    	_lhs->evaluateJacobians(outJacobians, applyChainRule * J_full);
        _rhs->evaluateJacobians(outJacobians, applyChainRule * _A_lhs);
    }

    // ----------------------------



    EuclideanExpressionNodeCrossEuclidean::EuclideanExpressionNodeCrossEuclidean(boost::shared_ptr<EuclideanExpressionNode> lhs, boost::shared_ptr<EuclideanExpressionNode> rhs) :
      _lhs(lhs), _rhs(rhs)
    {

    }

    EuclideanExpressionNodeCrossEuclidean::~EuclideanExpressionNodeCrossEuclidean()
    {

    }


    void EuclideanExpressionNodeCrossEuclidean::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _lhs->getDesignVariables(designVariables);
      _rhs->getDesignVariables(designVariables);
    }


    Eigen::Vector3d EuclideanExpressionNodeCrossEuclidean::toEuclideanImplementation() const
    {
      return sm::kinematics::crossMx(_lhs->toEuclidean()) * _rhs->toEuclidean();;
    }

    void EuclideanExpressionNodeCrossEuclidean::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      _lhs->evaluateJacobians(outJacobians, - sm::kinematics::crossMx(_rhs->toEuclidean()));
      _rhs->evaluateJacobians(outJacobians, sm::kinematics::crossMx(_lhs->toEuclidean()));
    }

    void EuclideanExpressionNodeCrossEuclidean::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      _lhs->evaluateJacobians(outJacobians, - applyChainRule * sm::kinematics::crossMx(_rhs->toEuclidean()));
      _rhs->evaluateJacobians(outJacobians, applyChainRule * sm::kinematics::crossMx(_lhs->toEuclidean()));
    }





    EuclideanExpressionNodeAddEuclidean::EuclideanExpressionNodeAddEuclidean(boost::shared_ptr<EuclideanExpressionNode> lhs, boost::shared_ptr<EuclideanExpressionNode> rhs) :
      _lhs(lhs), _rhs(rhs)
    {

    }

    EuclideanExpressionNodeAddEuclidean::~EuclideanExpressionNodeAddEuclidean()
    {

    }


    void EuclideanExpressionNodeAddEuclidean::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _lhs->getDesignVariables(designVariables);
      _rhs->getDesignVariables(designVariables);
    }


    Eigen::Vector3d EuclideanExpressionNodeAddEuclidean::toEuclideanImplementation() const
    {
      return _lhs->toEuclidean() + _rhs->toEuclidean();
    }

    void EuclideanExpressionNodeAddEuclidean::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      _lhs->evaluateJacobians(outJacobians);
      _rhs->evaluateJacobians(outJacobians);
    }

    void EuclideanExpressionNodeAddEuclidean::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      _lhs->evaluateJacobians(outJacobians, applyChainRule);
      _rhs->evaluateJacobians(outJacobians, applyChainRule);
    }





    EuclideanExpressionNodeSubtractEuclidean::EuclideanExpressionNodeSubtractEuclidean(boost::shared_ptr<EuclideanExpressionNode> lhs, boost::shared_ptr<EuclideanExpressionNode> rhs) :
      _lhs(lhs), _rhs(rhs)
    {

    }

    EuclideanExpressionNodeSubtractEuclidean::~EuclideanExpressionNodeSubtractEuclidean()
    {

    }


    void EuclideanExpressionNodeSubtractEuclidean::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _lhs->getDesignVariables(designVariables);
      _rhs->getDesignVariables(designVariables);
    }


    Eigen::Vector3d EuclideanExpressionNodeSubtractEuclidean::toEuclideanImplementation() const
    {
      return _lhs->toEuclidean() - _rhs->toEuclidean();
    }

    void EuclideanExpressionNodeSubtractEuclidean::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      _lhs->evaluateJacobians(outJacobians);
      _rhs->evaluateJacobians(outJacobians, -Eigen::Matrix3d::Identity());
    }

    void EuclideanExpressionNodeSubtractEuclidean::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      _lhs->evaluateJacobians(outJacobians, applyChainRule);
      _rhs->evaluateJacobians(outJacobians, - applyChainRule);
    }



    EuclideanExpressionNodeConstant::EuclideanExpressionNodeConstant(const Eigen::Vector3d & p) :
      _p(p)
    {
    }

    EuclideanExpressionNodeConstant::~EuclideanExpressionNodeConstant()
    {
    }

  void EuclideanExpressionNodeConstant::getDesignVariablesImplementation(DesignVariable::set_t & /* designVariables */) const
    {
    }

    Eigen::Vector3d EuclideanExpressionNodeConstant::toEuclideanImplementation() const
    {
      return _p;
    }

  void EuclideanExpressionNodeConstant::evaluateJacobiansImplementation(JacobianContainer & /* outJacobians */) const
    {
    }

  void EuclideanExpressionNodeConstant::evaluateJacobiansImplementation(JacobianContainer & /* outJacobians */, const Eigen::MatrixXd & /* applyChainRule */) const
    {
    }



    EuclideanExpressionNodeSubtractVector::EuclideanExpressionNodeSubtractVector(boost::shared_ptr<EuclideanExpressionNode> lhs, const Eigen::Vector3d & rhs) :
      _lhs(lhs), _rhs(rhs)
    {

    }

    EuclideanExpressionNodeSubtractVector::~EuclideanExpressionNodeSubtractVector()
    {

    }

    void EuclideanExpressionNodeSubtractVector::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _lhs->getDesignVariables(designVariables);
    }

    Eigen::Vector3d EuclideanExpressionNodeSubtractVector::toEuclideanImplementation() const
    {
      return _lhs->toEuclidean() - _rhs;
    }

    void EuclideanExpressionNodeSubtractVector::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      _lhs->evaluateJacobians(outJacobians);
    }

    void EuclideanExpressionNodeSubtractVector::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      _lhs->evaluateJacobians(outJacobians, applyChainRule);
    }



    EuclideanExpressionNodeNegated::EuclideanExpressionNodeNegated(boost::shared_ptr<EuclideanExpressionNode> operand) :
      _operand(operand)
    {

    }

    EuclideanExpressionNodeNegated::~EuclideanExpressionNodeNegated()
    {
    }

    void EuclideanExpressionNodeNegated::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _operand->getDesignVariables(designVariables);
    }

    Eigen::Vector3d EuclideanExpressionNodeNegated::toEuclideanImplementation() const
    {
      return - _operand->toEuclidean();
    }

    void EuclideanExpressionNodeNegated::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      _operand->evaluateJacobians(outJacobians, -Eigen::Matrix3d::Identity());
    }

    void EuclideanExpressionNodeNegated::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      _operand->evaluateJacobians(outJacobians, -applyChainRule);
    }


    EuclideanExpressionNodeScalarMultiply::EuclideanExpressionNodeScalarMultiply(boost::shared_ptr<EuclideanExpressionNode> p, boost::shared_ptr<ScalarExpressionNode> s) :
      _p(p),
      _s(s)
    {

    }

    EuclideanExpressionNodeScalarMultiply::~EuclideanExpressionNodeScalarMultiply()
    {
    }

    void EuclideanExpressionNodeScalarMultiply::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _p->getDesignVariables(designVariables);
      _s->getDesignVariables(designVariables);
    }

    Eigen::Vector3d EuclideanExpressionNodeScalarMultiply::toEuclideanImplementation() const
    {
      return _p->toEuclidean() * _s->toScalar();
    }

    void EuclideanExpressionNodeScalarMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      _p->evaluateJacobians(outJacobians, Eigen::Matrix3d::Identity() * _s->toScalar());
      _s->evaluateJacobians(outJacobians, _p->toEuclidean());
    }

    void EuclideanExpressionNodeScalarMultiply::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      _p->evaluateJacobians(outJacobians, applyChainRule * _s->toScalar());
      _s->evaluateJacobians(outJacobians, applyChainRule * _p->toEuclidean());
    }


    VectorExpression2EuclideanExpressionAdapter::VectorExpression2EuclideanExpressionAdapter(boost::shared_ptr<VectorExpressionNode<3> > vectorExpressionNode) :
      _vectorExpressionNode(vectorExpressionNode)
    {

    }

    VectorExpression2EuclideanExpressionAdapter::~VectorExpression2EuclideanExpressionAdapter()
    {
    }

    void VectorExpression2EuclideanExpressionAdapter::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _vectorExpressionNode->getDesignVariables(designVariables);
    }

    Eigen::Vector3d VectorExpression2EuclideanExpressionAdapter::toEuclideanImplementation() const
    {
      return _vectorExpressionNode->toVector();
    }

    void VectorExpression2EuclideanExpressionAdapter::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      _vectorExpressionNode->evaluateJacobians(outJacobians, Eigen::Matrix3d::Identity());
    }

    void VectorExpression2EuclideanExpressionAdapter::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      _vectorExpressionNode->evaluateJacobians(outJacobians, applyChainRule * Eigen::Matrix3d::Identity());
    }

  
  EuclideanExpressionNodeTranslation::EuclideanExpressionNodeTranslation(boost::shared_ptr<TransformationExpressionNode> operand) :
      _operand(operand) {
    
  }

  EuclideanExpressionNodeTranslation::~EuclideanExpressionNodeTranslation() {

  }


  Eigen::Vector3d EuclideanExpressionNodeTranslation::toEuclideanImplementation() const {
    return _operand->toTransformationMatrix().topRightCorner<3,1>();
  }

  void EuclideanExpressionNodeTranslation::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const {
    Eigen::MatrixXd J = Eigen::MatrixXd::Identity(3,6);
    Eigen::Vector3d p = _operand->toTransformationMatrix().topRightCorner<3,1>();
    J.topRightCorner<3,3>() = sm::kinematics::crossMx(p);
    _operand->evaluateJacobians(outJacobians, J);
  }

  void EuclideanExpressionNodeTranslation::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const {
    Eigen::MatrixXd J = Eigen::MatrixXd::Identity(3,6);
    Eigen::Vector3d p = _operand->toTransformationMatrix().topRightCorner<3,1>();
    J.topRightCorner<3,3>() = sm::kinematics::crossMx(p);
    _operand->evaluateJacobians(outJacobians, applyChainRule*J);
  }

  void EuclideanExpressionNodeTranslation::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const {
    return _operand->getDesignVariables(designVariables); 
  }



  EuclideanExpressionNodeRotationParameters::EuclideanExpressionNodeRotationParameters(boost::shared_ptr<RotationExpressionNode> operand, sm::kinematics::RotationalKinematics::Ptr rk) :
      _operand(operand), _rk(rk) {

  }

  EuclideanExpressionNodeRotationParameters::~EuclideanExpressionNodeRotationParameters() {

  }


  Eigen::Vector3d EuclideanExpressionNodeRotationParameters::toEuclideanImplementation() const {
    return _rk->rotationMatrixToParameters(_operand->toRotationMatrix());
  }

  void EuclideanExpressionNodeRotationParameters::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const {
    Eigen::MatrixXd J = _rk->parametersToSMatrix(
        _rk->rotationMatrixToParameters(_operand->toRotationMatrix())).inverse();
    _operand->evaluateJacobians(outJacobians,J);

  }

  void EuclideanExpressionNodeRotationParameters::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const {
    Eigen::MatrixXd J = _rk->parametersToSMatrix(
        _rk->rotationMatrixToParameters(_operand->toRotationMatrix())).inverse();
    _operand->evaluateJacobians(outJacobians, applyChainRule*J);

  }

  void EuclideanExpressionNodeRotationParameters::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const {
    return _operand->getDesignVariables(designVariables);
  }


  EuclideanExpressionNodeFromHomogeneous::EuclideanExpressionNodeFromHomogeneous(boost::shared_ptr<HomogeneousExpressionNode> root) : _root(root) {

  }
  EuclideanExpressionNodeFromHomogeneous:: ~EuclideanExpressionNodeFromHomogeneous() {

  }


  Eigen::Vector3d EuclideanExpressionNodeFromHomogeneous::toEuclideanImplementation() const {
    return sm::kinematics::fromHomogeneous( _root->toHomogeneous() );
  }

  void EuclideanExpressionNodeFromHomogeneous::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const {
    //Eigen::Vector3d fromHomogeneous(const Eigen::Vector4d & v, Eigen::Matrix<double,3,4> * jacobian = NULL);
    Eigen::Matrix<double,3,4> Jh;
    sm::kinematics::fromHomogeneous( _root->toHomogeneous(), &Jh );
    _root->evaluateJacobians( outJacobians, Jh );
  }

  void EuclideanExpressionNodeFromHomogeneous::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const {
    Eigen::Matrix<double,3,4> Jh;
    sm::kinematics::fromHomogeneous( _root->toHomogeneous(), &Jh );
    _root->evaluateJacobians( outJacobians, applyChainRule * Jh );

  }

  void EuclideanExpressionNodeFromHomogeneous::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const {
    _root->getDesignVariables(designVariables);
  }

    EuclideanExpressionNodeElementwiseMultiplyEuclidean::EuclideanExpressionNodeElementwiseMultiplyEuclidean(boost::shared_ptr<EuclideanExpressionNode> lhs, boost::shared_ptr<EuclideanExpressionNode> rhs) :
      _lhs(lhs), _rhs(rhs)
    {

    }

    EuclideanExpressionNodeElementwiseMultiplyEuclidean::~EuclideanExpressionNodeElementwiseMultiplyEuclidean()
    {

    }


    void EuclideanExpressionNodeElementwiseMultiplyEuclidean::getDesignVariablesImplementation(DesignVariable::set_t & designVariables) const
    {
      _lhs->getDesignVariables(designVariables);
      _rhs->getDesignVariables(designVariables);
    }


    Eigen::Vector3d EuclideanExpressionNodeElementwiseMultiplyEuclidean::toEuclideanImplementation() const
    {
      return (_lhs->toEuclidean()).cwiseProduct(_rhs->toEuclidean());
    }

    void EuclideanExpressionNodeElementwiseMultiplyEuclidean::evaluateJacobiansImplementation(JacobianContainer & outJacobians) const
    {
      _lhs->evaluateJacobians(outJacobians, _rhs->toEuclidean().asDiagonal());
      _rhs->evaluateJacobians(outJacobians, _lhs->toEuclidean().asDiagonal());
    }

    void EuclideanExpressionNodeElementwiseMultiplyEuclidean::evaluateJacobiansImplementation(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
    {
      _lhs->evaluateJacobians(outJacobians, applyChainRule * _rhs->toEuclidean().asDiagonal());
      _rhs->evaluateJacobians(outJacobians, applyChainRule * _lhs->toEuclidean().asDiagonal());
    }

  
  } // namespace backend
} // namespace aslam
