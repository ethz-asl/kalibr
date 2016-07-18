namespace aslam {
  namespace backend {

    template<typename CAMERA_T>
    ScalarExpressionNodeKeypointTime<CAMERA_T>::ScalarExpressionNodeKeypointTime(
        const aslam::Time & stamp,
        const Eigen::VectorXd & y,
        boost::shared_ptr<backend::CameraDesignVariable<CAMERA_T> > dv
    )
        : _stamp(stamp),
          _y(y),
          _dv(dv)
    {
      SM_ASSERT_TRUE(std::runtime_error, _dv.get() != NULL,
                     "Unable to intialize with a null design variable");
    }

    template<typename CAMERA_T>
    ScalarExpressionNodeKeypointTime<CAMERA_T>::~ScalarExpressionNodeKeypointTime() {

    }

    template<typename CAMERA_T>
    double ScalarExpressionNodeKeypointTime<CAMERA_T>::toScalarImplementation() const {
      return (_stamp + _dv->camera()->temporalOffset(_y)).toSec();
    }

    template<typename CAMERA_T>
    void ScalarExpressionNodeKeypointTime<CAMERA_T>::evaluateJacobiansImplementation(
        backend::JacobianContainer & outJacobians
    ) const{
      Eigen::MatrixXd Ji;

      Ji.resize(1, 1);
      Ji(0, 0) = _y[1];
      outJacobians.add(_dv->shutterDesignVariable().get(), Ji);
    }

    template<typename CAMERA_T>
    void ScalarExpressionNodeKeypointTime<CAMERA_T>::evaluateJacobiansImplementation(
        backend::JacobianContainer & outJacobians,
        const Eigen::MatrixXd & applyChainRule
    ) const {
      Eigen::MatrixXd Ji;

      Ji.resize(1, 1);
      Ji(0, 0) = _y[1];
      outJacobians.add(_dv->shutterDesignVariable().get(), applyChainRule * Ji);
    }

    template<typename CAMERA_T>
    void ScalarExpressionNodeKeypointTime<CAMERA_T>::getDesignVariablesImplementation(
        backend::DesignVariable::set_t & designVariables
    ) const {
      designVariables.insert(_dv->shutterDesignVariable().get());
    }

  } // namespace backend
}  // namespace aslam
