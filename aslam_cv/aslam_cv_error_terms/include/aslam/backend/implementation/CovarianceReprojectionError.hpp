
namespace aslam {
  namespace backend {
    
    template<typename F>
    CovarianceReprojectionError<F>::CovarianceReprojectionError()
    {

    }


    template<typename F>
    CovarianceReprojectionError<F>::CovarianceReprojectionError(
      const frame_t * frame,
      int keypointIndex,
			const HomogeneousExpression & point,
      CameraDesignVariable<camera_geometry_t> camera,
      spline_t* spline
    ) :
      _frame(frame),
      _keypointIndex(keypointIndex),
      _point(point),
      _camera(camera),
      _spline(spline)
    {
      SM_ASSERT_TRUE(Exception, frame != NULL, "The frame must not be null");
      SM_ASSERT_TRUE(Exception, _frame->numKeypoints() > _keypointIndex, "Keypoint index must be in bounds of frame.");

      // if a spline is given, estimate the covariance in each iteration
      //if(!spline)
      //    parent_t::_invR = _frame->keypoint(_keypointIndex).invR();
      JacobianContainer::set_t dvs;
      point.getDesignVariables(dvs);	// point dv's
      camera.getDesignVariables(dvs);	// camera dv's

      parent_t::setDesignVariablesIterator(dvs.begin(), dvs.end());
    }


    template<typename F>
    CovarianceReprojectionError<F>::~CovarianceReprojectionError()
    {

    }

    template<typename F>
    Eigen::MatrixXd CovarianceReprojectionError<F>::covarianceMap() {

    	const keypoint_t & k = _frame->keypoint(_keypointIndex);
    	const camera_geometry_t & cam = _frame->geometry();

    	Eigen::Vector4d p = _point.toHomogeneous();
    	measurement_t hat_y;
    	Eigen::Matrix<double, 2,4> outJp;
    	cam.homogeneousToKeypoint(p, hat_y, outJp);

    	double lineDelay = cam.shutter().lineDelay();

    	double observationTime = this->observationTime();

    	Eigen::VectorXd splinePoint = _spline->spline().evalD(observationTime, 0);
    	// evaluate the covariance:
    	Eigen::MatrixXd JT;
    	Eigen::VectorXd Phi_dot_c = _spline->spline().evalD(observationTime,1); // phi_dot * c (t_0)
    	Eigen::MatrixXd T = _spline->spline().curveValueToTransformationAndJacobian(splinePoint, &JT);

    	Eigen::MatrixXd TPboxminus = sm::kinematics::boxMinus(T*p);

    	//std::cout << "TPboxminus:" << std::endl << TPboxminus << std::endl;

    	Eigen::MatrixXd J = outJp * TPboxminus * JT * Phi_dot_c * lineDelay; //outJp * J_t;
    	//std::cout << "J: " << std::endl << J << std::endl;

    	Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2,2);

    	A(0,1) += J(0);
    	A(1,1) += J(1);

        // make sure, the variance of v remains positive and set to 0 if it is negative

       // if (A(1,1) < 0) {
       //     std::cout << "set one of them to zero..." << std::endl;
       //     A(1,1) = 0; ======> THIS IS BAD!
       // }

    	return A;

    }

    template<typename F>
    double CovarianceReprojectionError<F>::evaluateErrorImplementation()
    {
      const keypoint_t & k = _frame->keypoint(_keypointIndex);
      const camera_geometry_t & cam = _frame->geometry();

      Eigen::Vector4d p = _point.toHomogeneous();
      measurement_t hat_y;
      Eigen::Matrix<double, 2,4> outJp;
	    cam.homogeneousToKeypoint(p, hat_y, outJp);

      parent_t::setError(k.y() - hat_y);

      if(_spline) {

    	  Eigen::MatrixXd A = covarianceMap();
    	  //std::cout << A << std::endl;

          parent_t::setInvR (((A*_frame->keypoint(_keypointIndex).invR().inverse()*A.transpose()).inverse())); // invert
    	  // setInvR(A*_frame->keypoint(_keypointIndex).invR()*A.transpose());

    	  // std::cout << "A: " << std::endl << A << std::endl ;
    	  //std::cout << "ARAT: " << std::endl << A*_frame->keypoint(_keypointIndex).invR()*A.transpose() << std::endl << std::endl;
      }

      return parent_t::error().dot(parent_t::invR() * parent_t::error());
    }

    template<typename F>
    double CovarianceReprojectionError<F>::observationTime()
    {
    	return _frame->keypointTime(_keypointIndex).toSec(); // + _frame->keypoint(_keypointIndex).y()(1) * lineDelay;
    }


    template<typename F>
    void CovarianceReprojectionError<F>::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & _jacobians) const
    {
      //const keypoint_t & k = _frame->keypoint(_keypointIndex);
      const camera_geometry_t & cam = _frame->geometry();

      Eigen::Vector4d p = _point.toHomogeneous();
      typename camera_geometry_t::jacobian_homogeneous_t J;
      measurement_t hat_y;
	    cam.homogeneousToKeypoint(p, hat_y, J);

      _point.evaluateJacobians(_jacobians, -J);

      _camera.evaluateJacobians(_jacobians, p);
    }


  } // namespace backend
} // namespace aslam
