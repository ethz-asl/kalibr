namespace aslam {
  namespace backend {


    template<typename T>
    DesignVariableAdapter<T>::DesignVariableAdapter(T* dv, bool adapterOwnsPointer)
    {
      if (adapterOwnsPointer) {
        _dv.reset(dv);
      } else {
        _dv.reset(dv, sm::null_deleter());
      }
    }

    template<typename T>
    DesignVariableAdapter<T>::DesignVariableAdapter(const boost::shared_ptr<T>& dv) : _dv(dv)
    {
    }

    template<typename T>
    DesignVariableAdapter<T>::~DesignVariableAdapter()
    {
    }

    /// \brief what is the number of dimensions of the perturbation variable.
    template<typename T>
    int DesignVariableAdapter<T>::minimalDimensionsImplementation() const
    {
      return _dv->minimalDimensions();
    }

    template<typename T>
    Eigen::MatrixXd DesignVariableAdapter<T>::getParameters()
    {
      Eigen::MatrixXd p;
      _dv->getParameters(p);
      return p;
    }


    /// \brief Update the design variable.
    template<typename T>
    void DesignVariableAdapter<T>::updateImplementation(const double* dp, int /* size */)
    {
      _dv->getParameters(_backup);
      _dv->update(dp);
    }


    /// \brief Revert the last state update.
    template<typename T>
    void DesignVariableAdapter<T>::revertUpdateImplementation()
    {
      _dv->setParameters(_backup);
    }

    template<typename T>
    T& DesignVariableAdapter<T>::value()
    {
      return *_dv;
    }
    template<typename T>
    const T& DesignVariableAdapter<T>::value() const
    {
      return *_dv;
    }

    template<typename T>
    boost::shared_ptr<T> DesignVariableAdapter<T>::valuePtr()
    {
      return _dv;
    }
    template<typename T>
    boost::shared_ptr<const T> DesignVariableAdapter<T>::constValuePtr() const
    {
      return _dv;
    }

    template<typename T>
    void DesignVariableAdapter<T>::getParametersImplementation(
        Eigen::MatrixXd& value) const {
      _dv->getParameters(value);
    }

    template<typename T>
    void DesignVariableAdapter<T>::setParametersImplementation(
        const Eigen::MatrixXd& value) {
      _dv->getParameters(_backup);
      _dv->setParameters(value);
    }

    template<typename T>
    void DesignVariableAdapter<T>::minimalDifferenceImplementation(const Eigen::MatrixXd& /* xHat */, Eigen::VectorXd& /* outDifference */) const
    {
    	// TODO: implement
    }

    template<typename T>
    void DesignVariableAdapter<T>::minimalDifferenceAndJacobianImplementation(const Eigen::MatrixXd& /* xHat */, Eigen::VectorXd& /* outDifference */, Eigen::MatrixXd& /* outJacobian */) const
    {
    	// TODO: implement
    }


  } // namespace backend
} // namespace aslam
