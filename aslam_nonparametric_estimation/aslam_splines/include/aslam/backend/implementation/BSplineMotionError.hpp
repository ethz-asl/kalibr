#include <aslam/backend/BSplineMotionError.hpp>
#include <stdio.h>
namespace aslam {
    namespace backend {
        

    
        template<class SPLINE_T>
        BSplineMotionError<SPLINE_T>::BSplineMotionError(spline_t * splineDV, Eigen::MatrixXd W):
        _splineDV(splineDV), _W(W)
        {
        	initialize(splineDV, W, 2);	// default: acceleration criterion
        }

        template<class SPLINE_T>
        BSplineMotionError<SPLINE_T>::BSplineMotionError(spline_t * splineDV, Eigen::MatrixXd W, unsigned int errorTermOrder):
        _splineDV(splineDV), _W(W)
        {
        	initialize(splineDV, W, errorTermOrder);
        }

        template<class SPLINE_T>
        BSplineMotionError<SPLINE_T>::~BSplineMotionError()
        {

        }

        template<class SPLINE_T>
        void BSplineMotionError<SPLINE_T>::initialize(spline_t * /* splineDV */, Eigen::MatrixXd /* W */, unsigned int errorTermOrder) {

        	// check spline order:
        	int splineOrder = _splineDV->spline().splineOrder();
        	if(splineOrder <= (int)errorTermOrder && splineOrder >= 2) {
        		errorTermOrder = splineOrder-1;
        		std::cout << "! Invalid ErrorTermOrder reduced to " << errorTermOrder << std::endl;
        	}
        	sbm_t Qsp = _splineDV->spline().curveQuadraticIntegralSparse(_W, errorTermOrder);

        	// set spline design variables
        	unsigned int numberOfSplineDesignVariables = _splineDV->spline().numVvCoefficients();
        	_coefficientVectorLength = _splineDV->spline().coefficients().rows() * numberOfSplineDesignVariables;

        	Qsp.cloneInto(_Q);
        	//      std::cout << "next:" << std::endl;

        	//      std::cout << Q.cols() << ":" << Q.rows() << std::endl;
        	//      std::cout << _Q->cols() << ":" << _Q->rows() << std::endl;

        	// Tell the super class about the design variables:
        	// loop the design variables and add to vector:
        	std::vector<aslam::backend::DesignVariable*> dvV;
        	for ( unsigned int i = 0; i < numberOfSplineDesignVariables; i++) {
        		dvV.push_back(_splineDV->designVariable(i));
        	}
        	setDesignVariables(dvV);

        }


        /// \brief evaluate the error term and return the weighted squared error e^T invR e
        template<class SPLINE_T>
        double BSplineMotionError<SPLINE_T>::evaluateErrorImplementation()
        {       
            // the error is a scalar: c' Q c, with c the vector valued spline coefficients stacked

            const double* cMat = &((_splineDV->spline()).coefficients()(0,0));
            Eigen::Map<const Eigen::VectorXd> c = Eigen::VectorXd::Map(cMat, _coefficientVectorLength);

            // Q*c :
            // create result container:
            Eigen::VectorXd Qc(_Q.rows());  // number of rows of Q:
            Qc.setZero();
           //  std::cout << Qc->rows() << ":" << Qc->cols() << std::endl;
            _Q.multiply(&Qc, c);
            
            return c.transpose() * (Qc);
        }


        /// \brief evaluate the jacobians
        template<class SPLINE_T>
        void BSplineMotionError<SPLINE_T>::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & /* _jacobians */) const
        {
                      
          // this is an error...
          SM_THROW(Exception, "This is currently unsupported");
          
        }
          
          
          
        template<class SPLINE_T>
        void BSplineMotionError<SPLINE_T>::buildHessianImplementation(SparseBlockMatrix & outHessian, Eigen::VectorXd & outRhs, bool /* useMEstimator */) {
            
            
            // get the coefficients:
            Eigen::MatrixXd coeff = _splineDV->spline().coefficients();
            // create a column vector of spline coefficients
            int dim = coeff.rows(); 
            int seg = coeff.cols();
            // build a vector of coefficients:
            Eigen::VectorXd c(dim*seg);
            // rows are spline dimension
            for(int i = 0; i < seg; i++) {
                c.block(i*dim,0,dim,1) = coeff.block(0, i, dim,1);
            }
            
            // right hand side:
            Eigen::VectorXd b_u(_Q.rows());  // number of rows of Q:
            
            b_u.setZero();
      /*      std::cout <<"b" << std::endl;
            for(int i = 0 ; i < b_u->rows(); i++)
                std::cout << (*b_u)(i) << std::endl;
                        std::cout <<"/b" << std::endl;  */ 
            
            _Q.multiply(&b_u, c);

            // place the hessian elements in the correct place:        
        
            // build hessian:
            for(size_t i = 0; i < numDesignVariables(); i++)
            {

            	if( designVariable(i)->isActive()) {

            		// get the block index
            		int colBlockIndex = designVariable(i)->blockIndex();
            		int rows = designVariable(i)->minimalDimensions();
            		int rowBase = outHessian.colBaseOfBlock(colBlockIndex);

            		// <- this is our column index
            		//_numberOfSplineDesignVariables
            		for(size_t j = 0; j <= i; j++) // upper triangle should be sufficient
            		{

            			if (designVariable(j)->isActive()) {

            				int rowBlockIndex = designVariable(j)->blockIndex();

            				// select the corresponding block in _Q:
            				Eigen::MatrixXd* Qblock = _Q.block(i,j, false);  // get block and do NOT allocate.

            				if (Qblock) { // check if block exists
            					// get the Hessian Block
            					const bool allocateIfMissing = true;
            					Eigen::MatrixXd *Hblock = outHessian.block(rowBlockIndex, colBlockIndex, allocateIfMissing);
            					*Hblock += *Qblock;  // insert!
            				}
            			}

            		}

            		outRhs.segment(rowBase, rows) -= b_u.segment(i*rows, rows);
            	}
            }
            
            
            //std::cout << "OutHessian" << outHessian.toDense() << std::endl;
            
            // show outRhs:
          //  for (int i = 0;  i < outRhs.rows(); i++)
           //     std::cout << outRhs(i) <<  " : " << (*b_u)(i) << std::endl;
  
            
        }
          
        
        template<class SPLINE_T>
        Eigen::VectorXd BSplineMotionError<SPLINE_T>::rhs() { 
            
        	const double* cMat = &((_splineDV->spline()).coefficients()(0,0));
        	Eigen::Map<const Eigen::VectorXd> c = Eigen::VectorXd::Map(cMat, _coefficientVectorLength);
            
            // right hand side:
            Eigen::VectorXd b_u(_Q.rows());  // number of rows of Q:
            b_u.setZero();
            _Q.multiply(&b_u, -c);
            
            return b_u;
        }
      


    } // namespace backend
} // namespace aslam
