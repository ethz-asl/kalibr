namespace aslam {
	namespace backend {

	template<typename CAMERA_T>
	CameraDesignVariable<CAMERA_T>::CameraDesignVariable(const boost::shared_ptr<CAMERA_T> & camera):
	_camera(camera)
	{
		// create the design variables:
		_projectionDv.reset(new DesignVariableAdapter<projection_t>(&_camera->projection(), false));
		_distortionDv.reset(new DesignVariableAdapter<distortion_t>(&((_camera->projection()).distortion()), false));
		_shutterDv.reset(new DesignVariableAdapter<shutter_t>(&_camera->shutter(), false));
	}

	template<typename CAMERA_T>
	CameraDesignVariable<CAMERA_T>::~CameraDesignVariable() {}

	template<typename CAMERA_T>
	void CameraDesignVariable<CAMERA_T>::setActive(bool p, bool d, bool s) {
		_projectionDv->setActive(p);
		_distortionDv->setActive(d);
		_shutterDv->setActive(s);
	}

	template<typename CAMERA_T>
	Eigen::VectorXd CameraDesignVariable<CAMERA_T>::euclideanToKeypoint(Eigen::Vector3d p) {
		keypoint_t kp;
		_camera->euclideanToKeypoint( p, kp);
		return kp;
	}

	template<typename CAMERA_T>
	Eigen::VectorXd CameraDesignVariable<CAMERA_T>::homogeneousToKeypoint(Eigen::Vector4d ph) {
		keypoint_t kp;
		_camera->homogeneousToKeypoint(ph, kp);
		return kp;
	}

	template<typename CAMERA_T>
	void CameraDesignVariable<CAMERA_T>::evaluateJacobians(JacobianContainer & outJacobians, Eigen::Vector4d ph) const {

		Eigen::MatrixXd Jp;
		Eigen::MatrixXd Jd;
		Eigen::MatrixXd Js;

		if(_projectionDv->isActive()) {
			_camera->homogeneousToKeypointIntrinsicsJacobian(ph, Jp);
			outJacobians.add(_projectionDv.get(), -Jp);
		}
		if(_distortionDv->isActive()) {
			_camera->homogeneousToKeypointDistortionJacobian(ph, Jd);
			outJacobians.add(_distortionDv.get(), -Jd);
		}

	}

	template<typename CAMERA_T>
	void CameraDesignVariable<CAMERA_T>::evaluateJacobians(JacobianContainer & outJacobians, const Eigen::MatrixXd & applyChainRule, Eigen::Vector4d ph) const {

		Eigen::MatrixXd Jp;
		Eigen::MatrixXd Jd;
		Eigen::MatrixXd Js;

		_camera->homogeneousToKeypointIntrinsicsJacobian(ph, Jp);
		_camera->homogeneousToKeypointDistortionJacobian(ph, Jd);
		_camera->homogeneousToKeypointShutterJacobian(ph, Js);

		outJacobians.add(_projectionDv.get(), -applyChainRule*Jp);
		outJacobians.add(_distortionDv.get(), -applyChainRule*Jd);
		outJacobians.add(_shutterDv.get(), -applyChainRule*Js);

	}

	template<typename CAMERA_T>
	void CameraDesignVariable<CAMERA_T>::getDesignVariables(DesignVariable::set_t & designVariables) const {

		// add the 3 camera DV: distortion, intrinsics, shutter
		designVariables.insert(_projectionDv.get());
		designVariables.insert(_distortionDv.get());
		designVariables.insert(_shutterDv.get());

	}

    template<typename CAMERA_T>
    backend::ScalarExpression CameraDesignVariable<CAMERA_T>::keypointTime(
        const aslam::Time & imageStamp,
        const Eigen::VectorXd & y
    ) {
      if (_shutterDv->isActive()) {
        boost::shared_ptr<backend::ScalarExpressionNode> root(
        	new ScalarExpressionNodeKeypointTime<CAMERA_T>(
        		imageStamp,
        		y,
        		this->shared_from_this()
        	)
        );
        return backend::ScalarExpression(root);
      } else {
        return backend::ScalarExpression(
            (imageStamp + _camera->temporalOffset(y)).toSec()
        );
      }
    }

    template<typename CAMERA_T>
    backend::ScalarExpression CameraDesignVariable<CAMERA_T>::temporalOffset(
        const Eigen::VectorXd & y) {
      return keypointTime(aslam::Time(), y);
    }


	}	// backend
}	// aslam
