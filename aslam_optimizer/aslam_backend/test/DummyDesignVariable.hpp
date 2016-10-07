#ifndef _DUMMYDESIGNVARIABLE_H_
#define _DUMMYDESIGNVARIABLE_H_

template<int MD = 3>
class DummyDesignVariable : public aslam::backend::DesignVariable {
public:
  DummyDesignVariable() {}
  virtual ~DummyDesignVariable() {}
protected:
  /// \brief Revert the last state update.
  virtual void revertUpdateImplementation() {}

  /// \brief Update the design variable.
  virtual void updateImplementation(const double* /* dp */, int /* size */) {}

  /// \brief what is the number of dimensions of the perturbation variable.
  virtual int minimalDimensionsImplementation() const {
    return MD;
  }

  /// Returns the content of the design variable
  virtual void getParametersImplementation(Eigen::MatrixXd& /* value */) const {
  }

  /// Sets the content of the design variable
  virtual void setParametersImplementation(const Eigen::MatrixXd& /* value */) {
  }

};



#endif /* _DUMMYDESIGNVARIABLE_H_ */
