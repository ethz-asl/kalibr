

namespace aslam {
    namespace splines {
    
        /// \brief this guy takes a copy.
        template<int D>
        BSplineDesignVariable<D>::BSplineDesignVariable(const bsplines::BSpline & bspline) :
            _bspline(bspline)
        {
            // here is where the magic happens.

            // Create all of the design variables as maps into the vector of spline coefficients.
            for(int i = 0; i < _bspline.numVvCoefficients(); ++i)
            {
                _designVariables.push_back( new aslam::backend::DesignVariableMappedVector<D>( _bspline.fixedSizeVvCoefficientVector<D>(i) ) );
            }
        }
    
        template<int D>
        BSplineDesignVariable<D>::~BSplineDesignVariable()
        {

        }
    
        /// \brief get the spline.
        template<int D>
        const bsplines::BSpline & BSplineDesignVariable<D>::spline()
        {
            return _bspline;
        }

        template<int D>
        size_t BSplineDesignVariable<D>::numDesignVariables()
        {
            return _designVariables.size();
        }

        template<int D>
        aslam::backend::DesignVariableMappedVector<D> * BSplineDesignVariable<D>::designVariable(size_t i)
        {
            SM_ASSERT_LT(aslam::Exception, i, _designVariables.size(), "Index out of bounds");
            return &_designVariables[i];
        }

        template<int D>
        aslam::backend::VectorExpression<D> BSplineDesignVariable<D>::toExpression(double tk, int derivativeOrder)
        {
            Eigen::VectorXi dvidxs = _bspline.localVvCoefficientVectorIndices(tk);
            std::vector<aslam::backend::DesignVariable *> dvs;
            for(int i = 0; i < dvidxs.size(); ++i)
            {
                dvs.push_back(&_designVariables[dvidxs[i]]);
            }
            boost::shared_ptr<aslam::splines::BSplineVectorExpressionNode<D> > root( new aslam::splines::BSplineVectorExpressionNode<D>(&_bspline, derivativeOrder, dvs, tk) );
      
            return aslam::backend::VectorExpression<D>(root);
      
        }

        template<int D>
        std::vector<aslam::backend::DesignVariable *> BSplineDesignVariable<D>::getDesignVariables(double tk) const
        {
            Eigen::VectorXi dvidxs = _bspline.localVvCoefficientVectorIndices(tk);
            std::vector<aslam::backend::DesignVariable *> dvs;
            for(int i = 0; i < dvidxs.size(); ++i)
            {
                dvs.push_back(const_cast<aslam::backend::DesignVariableMappedVector<D>*>(&_designVariables[dvidxs[i]]));
            }

            return dvs;
        }


        template<int D>
        void BSplineDesignVariable<D>::addSegment(double t, const Eigen::VectorXd & p)
        {
            _bspline.addCurveSegment(t, p);
            _designVariables.push_back( new aslam::backend::DesignVariableMappedVector<D>( _bspline.fixedSizeVvCoefficientVector<D>(_bspline.numVvCoefficients()-1) ) );
            for(int i = 0; i < _bspline.numVvCoefficients()-1; i++)
            {
                _designVariables[i].updateMap(_bspline.fixedSizeVvCoefficientVector<D>(i).data());
            }
        }


        template<int D>
        void BSplineDesignVariable<D>::addSegment2(double t, const Eigen::VectorXd & p, double lambda)
        {
            _bspline.addCurveSegment2(t,p,lambda);
            _designVariables.push_back( new aslam::backend::DesignVariableMappedVector<D>( _bspline.fixedSizeVvCoefficientVector<D>(_bspline.numVvCoefficients()-1) ) );
            for(int i = 0; i < _bspline.numVvCoefficients()-1; i++)
            {
                _designVariables[i].updateMap(_bspline.fixedSizeVvCoefficientVector<D>(i).data());
            }    }


        template<int D>
        void BSplineDesignVariable<D>::removeSegment()
        {
            _bspline.removeCurveSegment();
            _designVariables.erase(_designVariables.begin());
            for(int i = 0; i < _bspline.numVvCoefficients(); i++)
            {
                _designVariables[i].updateMap(_bspline.fixedSizeVvCoefficientVector<D>(i).data());
            }
        }

    } // namespace splines
} // namespace aslam
