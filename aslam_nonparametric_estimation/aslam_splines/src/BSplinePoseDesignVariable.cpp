#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/splines/BSplineExpressions.hpp>


namespace aslam {
    namespace splines {
    
        /// \brief this guy takes a copy.
        BSplinePoseDesignVariable::BSplinePoseDesignVariable(const bsplines::BSplinePose & bsplinePose) :
            _bsplinePose(bsplinePose)
        {
            // here is where the magic happens.

            // Create all of the design variables as maps into the vector of spline coefficients.
            for(int i = 0; i < _bsplinePose.numVvCoefficients(); ++i)
            {
                _designVariables.push_back( new aslam::backend::DesignVariableMappedVector<6>( _bsplinePose.fixedSizeVvCoefficientVector<6>(i) ) );
            }
        }
    
        BSplinePoseDesignVariable::~BSplinePoseDesignVariable()
        {

        }
    
        /// \brief get the spline.
        const bsplines::BSplinePose & BSplinePoseDesignVariable::spline()
        {
            return _bsplinePose;
        }

        size_t BSplinePoseDesignVariable::numDesignVariables()
        {
            return _designVariables.size();
        }

        aslam::backend::DesignVariableMappedVector<6> * BSplinePoseDesignVariable::designVariable(size_t i)
        {
            SM_ASSERT_LT(aslam::Exception, i, _designVariables.size(), "Index out of bounds");
            return &_designVariables[i];
        }

        Eigen::VectorXi BSplinePoseDesignVariable::getActiveDesignVariableIndices(double tk)
        {
            return _bsplinePose.localVvCoefficientVectorIndices(tk);
        }

        std::vector<aslam::backend::DesignVariable *> BSplinePoseDesignVariable::getDesignVariables(double tk)
        {
            Eigen::VectorXi dvidxs = _bsplinePose.localVvCoefficientVectorIndices(tk);
            std::vector<aslam::backend::DesignVariable *> dvs;
            for(int i = 0; i < dvidxs.size(); ++i)
            {
                dvs.push_back(const_cast<aslam::backend::DesignVariableMappedVector<6>*>(&_designVariables[dvidxs[i]]));
            }

            return dvs;
        }

        aslam::backend::TransformationExpression BSplinePoseDesignVariable::transformation(double tk)
        {
      
            Eigen::VectorXi dvidxs = _bsplinePose.localVvCoefficientVectorIndices(tk);
            std::vector<aslam::backend::DesignVariable *> dvs;
            for(int i = 0; i < dvidxs.size(); ++i)
            {
                dvs.push_back(&_designVariables[dvidxs[i]]);
            }
      
            boost::shared_ptr<BSplineTransformationExpressionNode> root( new BSplineTransformationExpressionNode(&_bsplinePose, dvs, tk) );
      
            return aslam::backend::TransformationExpression(root);

        }
    
        aslam::backend::RotationExpression BSplinePoseDesignVariable::orientation(double tk)
        {
            Eigen::VectorXi dvidxs = _bsplinePose.localVvCoefficientVectorIndices(tk);
            std::vector<aslam::backend::DesignVariable *> dvs;
            for(int i = 0; i < dvidxs.size(); ++i)
            {
                dvs.push_back(&_designVariables[dvidxs[i]]);
            }
      
            boost::shared_ptr<BSplineRotationExpressionNode> root( new BSplineRotationExpressionNode(&_bsplinePose, dvs, tk) );
      
            return aslam::backend::RotationExpression(root);
      
        }

        aslam::backend::EuclideanExpression BSplinePoseDesignVariable::position(double tk)
        {
            Eigen::VectorXi dvidxs = _bsplinePose.localVvCoefficientVectorIndices(tk);
            std::vector<aslam::backend::DesignVariable *> dvs;
            for(int i = 0; i < dvidxs.size(); ++i)
            {
                dvs.push_back(&_designVariables[dvidxs[i]]);
            }
	
            boost::shared_ptr<BSplinePositionExpressionNode> root( new BSplinePositionExpressionNode(&_bsplinePose, dvs, tk) );
	
            return aslam::backend::EuclideanExpression(root);

        }

        aslam::backend::EuclideanExpression BSplinePoseDesignVariable::linearVelocity(double tk)
        {
            Eigen::VectorXi dvidxs = _bsplinePose.localVvCoefficientVectorIndices(tk);
            std::vector<aslam::backend::DesignVariable *> dvs;
            for(int i = 0; i < dvidxs.size(); ++i)
            {
                dvs.push_back(&_designVariables[dvidxs[i]]);
            }

            boost::shared_ptr<BSplineVelocityExpressionNode> root( new BSplineVelocityExpressionNode(&_bsplinePose, dvs, tk) );

            return aslam::backend::EuclideanExpression(root);

        }

        aslam::backend::EuclideanExpression BSplinePoseDesignVariable::linearAcceleration(double tk)
        {
            Eigen::VectorXi dvidxs = _bsplinePose.localVvCoefficientVectorIndices(tk);
            std::vector<aslam::backend::DesignVariable *> dvs;
            for(int i = 0; i < dvidxs.size(); ++i)
            {
                dvs.push_back(&_designVariables[dvidxs[i]]);
            }
	
            boost::shared_ptr<BSplineAccelerationExpressionNode> root( new BSplineAccelerationExpressionNode(&_bsplinePose, dvs, tk) );
	
            return aslam::backend::EuclideanExpression(root);

        }

      aslam::backend::EuclideanExpression
          BSplinePoseDesignVariable::linearAccelerationBodyFrame(double tk) {
        Eigen::VectorXi dvidxs =
          _bsplinePose.localVvCoefficientVectorIndices(tk);
        std::vector<aslam::backend::DesignVariable*> dvs;
        for (int i = 0; i < dvidxs.size(); ++i)
          dvs.push_back(&_designVariables[dvidxs[i]]);
        boost::shared_ptr<BSplineAccelerationBodyFrameExpressionNode> root(
          new BSplineAccelerationBodyFrameExpressionNode(
          &_bsplinePose, dvs, tk));
        return aslam::backend::EuclideanExpression(root);
      }

        aslam::backend::EuclideanExpression BSplinePoseDesignVariable::angularVelocityBodyFrame(double tk)
        {
            Eigen::VectorXi dvidxs = _bsplinePose.localVvCoefficientVectorIndices(tk);
            std::vector<aslam::backend::DesignVariable *> dvs;
            for(int i = 0; i < dvidxs.size(); ++i)
            {
                dvs.push_back(&_designVariables[dvidxs[i]]);
            }
	
            boost::shared_ptr<BSplineAngularVelocityBodyFrameExpressionNode> root( new BSplineAngularVelocityBodyFrameExpressionNode(&_bsplinePose, dvs, tk) );
	
            return aslam::backend::EuclideanExpression(root);

        }

        aslam::backend::EuclideanExpression BSplinePoseDesignVariable::angularAccelerationBodyFrame(double tk)
        {
        	Eigen::VectorXi dvidxs = _bsplinePose.localVvCoefficientVectorIndices(tk);
        	std::vector<aslam::backend::DesignVariable *> dvs;
        	for(int i = 0; i < dvidxs.size(); ++i)
        	{
        		dvs.push_back(&_designVariables[dvidxs[i]]);
        	}

        	boost::shared_ptr<BSplineAngularAccelerationBodyFrameExpressionNode> root( new BSplineAngularAccelerationBodyFrameExpressionNode(&_bsplinePose, dvs, tk) );

        	return aslam::backend::EuclideanExpression(root);

        }

        void BSplinePoseDesignVariable::addSegment(double t, Eigen::Matrix4d T)
        {
            _bsplinePose.addPoseSegment(t,T);
            _designVariables.push_back( new aslam::backend::DesignVariableMappedVector<6>( _bsplinePose.fixedSizeVvCoefficientVector<6>(_bsplinePose.numVvCoefficients()-1) ) );
            for(int i = 0; i < _bsplinePose.numVvCoefficients()-1; i++)
            {
                _designVariables[i].updateMap(_bsplinePose.fixedSizeVvCoefficientVector<6>(i).data());
            }
        }


        void BSplinePoseDesignVariable::addSegment2(double t, Eigen::Matrix4d T, double lambda)
        {
            _bsplinePose.addPoseSegment2(t,T,lambda);
            _designVariables.push_back( new aslam::backend::DesignVariableMappedVector<6>( _bsplinePose.fixedSizeVvCoefficientVector<6>(_bsplinePose.numVvCoefficients()-1) ) );
            for(int i = 0; i < _bsplinePose.numVvCoefficients()-1; i++)
            {
                _designVariables[i].updateMap(_bsplinePose.fixedSizeVvCoefficientVector<6>(i).data());
            }    
        }


        void BSplinePoseDesignVariable::removeSegment()
        {
            _bsplinePose.removeCurveSegment();
            _designVariables.erase(_designVariables.begin());
            for(int i = 0; i < _bsplinePose.numVvCoefficients(); i++)
            {
                _designVariables[i].updateMap(_bsplinePose.fixedSizeVvCoefficientVector<6>(i).data());
            }
        }

        aslam::backend::TransformationExpression BSplinePoseDesignVariable::transformationAtTime(const aslam::backend::ScalarExpression & time)
        {
        	return transformationAtTime(time, 0.0,0.0);
        }

        aslam::backend::TransformationExpression BSplinePoseDesignVariable::transformationAtTime(const aslam::backend::ScalarExpression & time, double leftBuffer, double rightBuffer)
        {
            boost::shared_ptr<TransformationTimeOffsetExpressionNode> root( new TransformationTimeOffsetExpressionNode( this, time, leftBuffer, rightBuffer) );
      
            return aslam::backend::TransformationExpression(root);

        }

    } // namespace splines
} // namespace aslam
