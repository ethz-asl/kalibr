#include <aslam/splines/BSplineExpressions.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>

namespace aslam {
    namespace splines {
    
        BSplineTransformationExpressionNode::BSplineTransformationExpressionNode(bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time) :
            _spline(spline), _designVariables(designVariables), _time(time)
        {

        }

        BSplineTransformationExpressionNode::~BSplineTransformationExpressionNode()
        {

        }

        Eigen::Matrix4d BSplineTransformationExpressionNode::toTransformationMatrixImplementation()
        {
            return _spline->transformation(_time);
        }

        void BSplineTransformationExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const
        {
            Eigen::MatrixXd J;
            _spline->transformationAndJacobian(_time, &J);
      
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], J.block<6,6>(0,i*6) );
            }
        }

        void BSplineTransformationExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            SM_ASSERT_EQ_DBG(aslam::Exception, applyChainRule.cols(), 6, "The chain rule matrix is the wrong size");

            Eigen::MatrixXd J;
            _spline->transformationAndJacobian(_time, &J);
      
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], applyChainRule * J.block<6,6>(0,i*6) );
            }

        }

        void BSplineTransformationExpressionNode::getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const
        {
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                designVariables.insert(_designVariables[i]);
            }

        }




        ///////////

        BSplineRotationExpressionNode::BSplineRotationExpressionNode(bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time) :
            _spline(spline), _designVariables(designVariables), _time(time)
        {

        }

        BSplineRotationExpressionNode::~BSplineRotationExpressionNode()
        {

        }

        Eigen::Matrix3d BSplineRotationExpressionNode::toRotationMatrixImplementation() const
        {
            return _spline->orientation(_time);
        }

        void BSplineRotationExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const
        {
            Eigen::MatrixXd J;
            _spline->orientationAndJacobian(_time, &J, NULL);

            SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 3, "Bad");
            SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(), 6 * (int)_designVariables.size(), "Bad");
      
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], J.block<3,6>(0,i*6) );
            }
        }

        void BSplineRotationExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            SM_ASSERT_EQ_DBG(aslam::Exception, applyChainRule.cols(), 3, "The chain rule matrix is the wrong size");

            Eigen::MatrixXd J;
            _spline->orientationAndJacobian(_time, &J, NULL);

            SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 3, "Bad");
            SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(), 6 * (int)_designVariables.size(), "Bad");
      
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], applyChainRule * J.block<3,6>(0,i*6) );
            }

        }

        void BSplineRotationExpressionNode::getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const
        {
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                designVariables.insert(_designVariables[i]);
            }

        }


        /////////////////////

        BSplinePositionExpressionNode::BSplinePositionExpressionNode(bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time) :
            _spline(spline), _designVariables(designVariables), _time(time)
        {

        }

        BSplinePositionExpressionNode::~BSplinePositionExpressionNode()
        {

        }

        Eigen::Vector3d BSplinePositionExpressionNode::toEuclideanImplementation() const
        {

            return _spline->eval(_time).head<3>();
        }

        void BSplinePositionExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const
        {
            Eigen::MatrixXd J;
            _spline->evalDAndJacobian(_time, 0, &J, NULL);
            SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 6, "Bad");
            SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(), 6 * (int)_designVariables.size(), "Bad");

            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], J.block<3,6>(0,i*6) );
            }
        }

        void BSplinePositionExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            SM_ASSERT_EQ_DBG(aslam::Exception, applyChainRule.cols(), 3, "The chain rule matrix is the wrong size");

            Eigen::MatrixXd J;
            _spline->evalDAndJacobian(_time, 0, &J, NULL);
            SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 6, "Bad");
            SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(), 6 * (int)_designVariables.size(), "Bad");
      
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], applyChainRule * J.block<3,6>(0,i*6) );
            }

        }

        void BSplinePositionExpressionNode::getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const
        {
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                designVariables.insert(_designVariables[i]);
            }

        }



        /////////////////////

        BSplineVelocityExpressionNode::BSplineVelocityExpressionNode(bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time) :
            _spline(spline), _designVariables(designVariables), _time(time)
        {

        }

        BSplineVelocityExpressionNode::~BSplineVelocityExpressionNode()
        {

        }

        Eigen::Vector3d BSplineVelocityExpressionNode::toEuclideanImplementation() const
        {

            return _spline->evalD(_time,1).head<3>();
        }

        void BSplineVelocityExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const
        {
            Eigen::MatrixXd J;
            _spline->evalDAndJacobian(_time, 1, &J, NULL);
            SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 6, "Bad");
            SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(), 6 * (int)_designVariables.size(), "Bad");

            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], J.block<3,6>(0,i*6) );
            }
        }

        void BSplineVelocityExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            SM_ASSERT_EQ_DBG(aslam::Exception, applyChainRule.cols(), 3, "The chain rule matrix is the wrong size");

            Eigen::MatrixXd J;
            _spline->evalDAndJacobian(_time, 1, &J, NULL);
            SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 6, "Bad");
            SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(), 6 * (int)_designVariables.size(), "Bad");
      
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], applyChainRule * J.block<3,6>(0,i*6) );
            }

        }

        void BSplineVelocityExpressionNode::getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const
        {
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                designVariables.insert(_designVariables[i]);
            }

        }




        /////////////////////

        BSplineAccelerationExpressionNode::BSplineAccelerationExpressionNode(bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time) :
            _spline(spline), _designVariables(designVariables), _time(time)
        {

        }

        BSplineAccelerationExpressionNode::~BSplineAccelerationExpressionNode()
        {

        }

        Eigen::Vector3d BSplineAccelerationExpressionNode::toEuclideanImplementation() const
        {

            return _spline->evalD(_time,2).head<3>();
        }

        void BSplineAccelerationExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const
        {
            Eigen::MatrixXd J;
            _spline->evalDAndJacobian(_time, 2, &J, NULL);
            SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 6, "Bad");
            SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(), 6 * (int)_designVariables.size(), "Bad");

            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], J.block<3,6>(0,i*6) );
            }
        }

        void BSplineAccelerationExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            SM_ASSERT_EQ_DBG(aslam::Exception, applyChainRule.cols(), 3, "The chain rule matrix is the wrong size");

            Eigen::MatrixXd J;
            _spline->evalDAndJacobian(_time, 2, &J, NULL);
            SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 6, "Bad");
            SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(), 6 * (int)_designVariables.size(), "Bad");
      
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], applyChainRule * J.block<3,6>(0,i*6) );
            }

        }

        void BSplineAccelerationExpressionNode::getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const
        {
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                designVariables.insert(_designVariables[i]);
            }

        }

    BSplineAccelerationBodyFrameExpressionNode::
        BSplineAccelerationBodyFrameExpressionNode(
        bsplines::BSplinePose* spline,
        const std::vector<aslam::backend::DesignVariable*>& designVariables,
        double time) :
        _spline(spline), _designVariables(designVariables), _time(time) {
    }

    BSplineAccelerationBodyFrameExpressionNode::
        ~BSplineAccelerationBodyFrameExpressionNode() {
    }

    Eigen::Vector3d BSplineAccelerationBodyFrameExpressionNode::
        toEuclideanImplementation() const {
        return _spline->linearAccelerationBodyFrame(_time);
    }

    void BSplineAccelerationBodyFrameExpressionNode::
        evaluateJacobiansImplementation(aslam::backend::JacobianContainer&
        outJacobians) const {
      Eigen::MatrixXd J;
      _spline->evalDAndJacobian(_time, 2, &J, NULL);
      SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 6, "Bad");
      SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(),
        6 * (int)_designVariables.size(), "Bad");
      for (size_t i = 0; i < _designVariables.size(); ++i)
        outJacobians.add(_designVariables[i], J.block<3, 6>(0, i * 6));
    }

    void BSplineAccelerationBodyFrameExpressionNode::
        evaluateJacobiansImplementation(aslam::backend::JacobianContainer&
        outJacobians, const Eigen::MatrixXd & applyChainRule) const {
      SM_ASSERT_EQ_DBG(aslam::Exception, applyChainRule.cols(), 3,
        "The chain rule matrix is the wrong size");
      Eigen::MatrixXd J;
      _spline->evalDAndJacobian(_time, 2, &J, NULL);
      SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 6, "Bad");
      SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(),
        6 * (int)_designVariables.size(), "Bad");
      for (size_t i = 0; i < _designVariables.size(); ++i)
        outJacobians.add(_designVariables[i],
          applyChainRule * J.block<3 , 6>(0, i * 6));
    }

    void BSplineAccelerationBodyFrameExpressionNode::
        getDesignVariablesImplementation(
        aslam::backend::DesignVariable::set_t& designVariables) const {
      for (size_t i = 0; i < _designVariables.size(); ++i)
        designVariables.insert(_designVariables[i]);
    }


        ///////////////////
        BSplineAngularVelocityBodyFrameExpressionNode::BSplineAngularVelocityBodyFrameExpressionNode(bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time) :
            _spline(spline), _designVariables(designVariables), _time(time)
        {

        }

        BSplineAngularVelocityBodyFrameExpressionNode::~BSplineAngularVelocityBodyFrameExpressionNode()
        {

        }

        Eigen::Vector3d BSplineAngularVelocityBodyFrameExpressionNode::toEuclideanImplementation() const
        {

            return _spline->angularVelocityBodyFrame(_time);
        }

        void BSplineAngularVelocityBodyFrameExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const
        {
            Eigen::MatrixXd J;
            _spline->angularVelocityBodyFrameAndJacobian(_time, &J, NULL);
            SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 3, "Bad");
            SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(), 6 * (int)_designVariables.size(), "Bad");

            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], J.block<3,6>(0,i*6) );
            }
        }

        void BSplineAngularVelocityBodyFrameExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            SM_ASSERT_EQ_DBG(aslam::Exception, applyChainRule.cols(), 3, "The chain rule matrix is the wrong size");

            Eigen::MatrixXd J;
            _spline->angularVelocityBodyFrameAndJacobian(_time, &J, NULL);
            SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 3, "Bad");
            SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(), 6 * (int)_designVariables.size(), "Bad");
      
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], applyChainRule * J.block<3,6>(0,i*6) );
            }

        }

        void BSplineAngularVelocityBodyFrameExpressionNode::getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const
        {
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                designVariables.insert(_designVariables[i]);
            }

        }


        ///////////////////////////////////
        // Time offset
        TransformationTimeOffsetExpressionNode::TransformationTimeOffsetExpressionNode(BSplinePoseDesignVariable * bspline, const aslam::backend::ScalarExpression & time, double bufferTmin, double bufferTmax) :
            _spline(bspline), _time(time)
        {

        	double initTime = time.toScalar();

        	if(bufferTmax + initTime > _spline->spline().t_max())
        		_bufferRight = _spline->spline().numValidTimeSegments()-1;
        	else
        		_bufferRight = _spline->spline().segmentIndex(initTime + bufferTmax);

        	if(initTime - bufferTmin < _spline->spline().t_min())
        		_bufferLeft = 0;
        	else
        		_bufferLeft = _spline->spline().segmentIndex(initTime - bufferTmin);

        	// take the full time span of the time segments
			_bufferTmax = _spline->spline().timeInterval(_bufferRight).second;
			_bufferTmin = _spline->spline().timeInterval(_bufferLeft).first;
			// store the local design variable indices over all segments:
			// the time in the center of the segment is taken to avoid numerical issues. maybe this is redundant
			Eigen::VectorXi leftCoeff = _spline->spline().localVvCoefficientVectorIndices( (_spline->spline().timeInterval(_bufferLeft).first + _spline->spline().timeInterval(_bufferLeft).second)/2.0 );
			Eigen::VectorXi rightCoeff = _spline->spline().localVvCoefficientVectorIndices( (_spline->spline().timeInterval(_bufferRight).first + _spline->spline().timeInterval(_bufferRight).second)/2.0 );

			// fill the vector with all the indices
			int l = leftCoeff(0);
			int r = rightCoeff(rightCoeff.size() -1);
			_localCoefficientIndices = Eigen::VectorXi(r - l + 1);
			for(int i = l; i <= r; i++)
				_localCoefficientIndices(i-l) = i;

        }
      
        TransformationTimeOffsetExpressionNode::~TransformationTimeOffsetExpressionNode()
        {
            
        }
        
        
        Eigen::Matrix4d TransformationTimeOffsetExpressionNode::toTransformationMatrixImplementation()
        {
        	SM_ASSERT_GE_LT(aslam::Exception, _time.toScalar(), _bufferTmin, _bufferTmax, "Spline Coefficient Buffer Exceeded. Set larger buffer margins!");
            return _spline->spline().transformation(_time.toScalar());
        }
      
        void TransformationTimeOffsetExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians)  const
        {
          
            Eigen::MatrixXd JT;
            Eigen::MatrixXd J;
          
          
            double observationTime = _time.toScalar();

            SM_ASSERT_GE_LT(aslam::Exception, observationTime, _bufferTmin, _bufferTmax, "Spline Coefficient Buffer Exceeded. Set larger buffer margins!");

            // the active indices
            Eigen::VectorXi dvidxs = _spline->spline().localVvCoefficientVectorIndices(observationTime);
            // nonzero Jacobians:
            _spline->spline().transformationAndJacobian(observationTime, &J);
            Eigen::MatrixXd JS;
            Eigen::VectorXd p;
          
            p = _spline->spline().evalDAndJacobian(observationTime,0,&JS, NULL);
           
          
            /*Eigen::Matrix4d T =*/ _spline->spline().curveValueToTransformationAndJacobian( p, &JT );    
            J = JT * JS;

            int minIdx = dvidxs(0);
            int maxIdx = dvidxs(dvidxs.size() - 1);

            for(int i = 0; i < _localCoefficientIndices.size(); i ++)
            {
            	// nonzero Jacobian
            	if(_localCoefficientIndices[i] >= minIdx && _localCoefficientIndices[i] <= maxIdx )
            		outJacobians.add(_spline->designVariable(_localCoefficientIndices[i]), J.block<6,6>(0,(i-minIdx)*6) );
            	// zero Jacobian:
            	else
            		outJacobians.add(_spline->designVariable(_localCoefficientIndices[i]), Eigen::Matrix<double, 6,6>::Zero());
            }

            // evaluate time derivative of the curves
            Eigen::VectorXd Phi_dot_c = _spline->spline().evalD(observationTime,1); // phi_dot * c (t_0)
            
            // Add the jacobians wrt line delay: \mbf S_T * \mbsdot \Phi_dot(t) * c * p_{i,v}
            _time.evaluateJacobians(outJacobians, JT * Phi_dot_c );
        }
      
        void TransformationTimeOffsetExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
          
            Eigen::MatrixXd JT;
            Eigen::MatrixXd J;
          
          
            double observationTime = _time.toScalar();
            SM_ASSERT_GE_LT(aslam::Exception, observationTime, _bufferTmin, _bufferTmax, "Spline Coefficient Buffer Exceeded. Set larger buffer margins!");


            Eigen::VectorXi dvidxs = _spline->spline().localVvCoefficientVectorIndices(observationTime);
            // _spline->spline().transformationAndJacobian(observationTime, &J);
            Eigen::MatrixXd JS;
            Eigen::VectorXd p;
          
            p = _spline->spline().evalDAndJacobian(observationTime,0,&JS, NULL);
           
          
            /*Eigen::Matrix4d T =*/ _spline->spline().curveValueToTransformationAndJacobian( p, &JT );    
            J = JT * JS;  
          
            int minIdx = dvidxs(0);
            int maxIdx = dvidxs(dvidxs.size() - 1);
            int j = 0;

            //std::cout << "dvidxs: " << std::endl << dvidxs << std::endl;
            //std::cout << "_localCoefficientIndices" << std::endl << _localCoefficientIndices << std::endl;

            for(int i = 0; i < _localCoefficientIndices.size(); i ++)
            {
            	// nonzero Jacobian
            	if(_localCoefficientIndices[i] >= minIdx && _localCoefficientIndices[i] <= maxIdx ) {
            		outJacobians.add(_spline->designVariable(_localCoefficientIndices[i]), applyChainRule * J.block<6,6>(0,(j)*6) );
            		j++;
            	}
            	// zero Jacobian:
            	else {
            		outJacobians.add(_spline->designVariable(_localCoefficientIndices[i]), applyChainRule * Eigen::Matrix<double, 6,6>::Zero());
            	}
            }

            // evaluate time derivative of the curves
            Eigen::VectorXd Phi_dot_c = _spline->spline().evalD(observationTime,1); // phi_dot * c (t_0)
          
          
            _time.evaluateJacobians(outJacobians, applyChainRule * JT * Phi_dot_c );
            
          
        }
      
        void TransformationTimeOffsetExpressionNode::getDesignVariablesImplementation(aslam::backend::JacobianContainer::set_t & designVariables) const
        {
            //double observationTime = _time.toScalar();
            //Eigen::VectorXi dvidxs = _spline->spline().localVvCoefficientVectorIndices(observationTime);
            for(int i = 0; i < _localCoefficientIndices.size(); ++i)
            {
                designVariables.insert( _spline->designVariable(_localCoefficientIndices[i]) );
            }
            _time.getDesignVariables(designVariables);
        }


        ///////////////////////////////////
        // EuclideanExpression offset

        BSplineEuclideanExpressionNode::BSplineEuclideanExpressionNode(bsplines::BSpline * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time, int order) :
            _spline(spline), _designVariables(designVariables), _time(time), _order(order)
        {

        }

        BSplineEuclideanExpressionNode::~BSplineEuclideanExpressionNode()
        {

        }

        Eigen::Vector3d BSplineEuclideanExpressionNode::toEuclideanImplementation() const
        {
            return _spline->evalD(_time, _order);
        }

        void BSplineEuclideanExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const
        {
            Eigen::MatrixXd J;
            _spline->evalDAndJacobian(_time, _order, &J, NULL);

            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], J.block<3,3>(0,i*3) );
            }

        }

        void BSplineEuclideanExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
            Eigen::MatrixXd J;
            _spline->evalDAndJacobian(_time, _order, &J, NULL);

            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                outJacobians.add(_designVariables[i], applyChainRule * J.block<3,3>(0,i*3) );
            }

        }

        void BSplineEuclideanExpressionNode::getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const
        {
            for(size_t i = 0; i < _designVariables.size(); ++i)
            {
                designVariables.insert(_designVariables[i]);
            }
            
        }

        BSplineAngularAccelerationBodyFrameExpressionNode::BSplineAngularAccelerationBodyFrameExpressionNode(bsplines::BSplinePose * spline, const std::vector<aslam::backend::DesignVariable *> & designVariables, double time) :
        _spline(spline), _designVariables(designVariables), _time(time)
        {

        }

        BSplineAngularAccelerationBodyFrameExpressionNode::~BSplineAngularAccelerationBodyFrameExpressionNode()
        {

        }

        Eigen::Vector3d BSplineAngularAccelerationBodyFrameExpressionNode::toEuclideanImplementation() const
        {

        	return _spline->angularAccelerationBodyFrame(_time);
        }

        void BSplineAngularAccelerationBodyFrameExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) const
        {
        	Eigen::MatrixXd J;
        	_spline->angularAccelerationBodyFrameAndJacobian(_time, &J, NULL);
        	SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 3, "Bad");
        	SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(), 6 * (int)_designVariables.size(), "Bad");

        	for(size_t i = 0; i < _designVariables.size(); ++i)
        	{
        		outJacobians.add(_designVariables[i], J.block<3,6>(0,i*6) );
        	}
        }

        void BSplineAngularAccelerationBodyFrameExpressionNode::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule) const
        {
        	SM_ASSERT_EQ_DBG(aslam::Exception, applyChainRule.cols(), 3, "The chain rule matrix is the wrong size");

        	Eigen::MatrixXd J;
        	_spline->angularAccelerationBodyFrameAndJacobian(_time, &J, NULL);
        	SM_ASSERT_EQ_DBG(aslam::Exception, J.rows(), 3, "Bad");
        	SM_ASSERT_EQ_DBG(aslam::Exception, J.cols(), 6 * (int)_designVariables.size(), "Bad");

        	for(size_t i = 0; i < _designVariables.size(); ++i)
        	{
        		outJacobians.add(_designVariables[i], applyChainRule * J.block<3,6>(0,i*6) );
        	}
        }

        void BSplineAngularAccelerationBodyFrameExpressionNode::getDesignVariablesImplementation(aslam::backend::DesignVariable::set_t & designVariables) const
        {
        	for(size_t i = 0; i < _designVariables.size(); ++i)
        	{
        		designVariables.insert(_designVariables[i]);
        	}
        }


    } // namespace splines
} // namespace aslam
