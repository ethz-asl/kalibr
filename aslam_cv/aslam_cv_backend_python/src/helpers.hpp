#ifndef _HELPERS_H_
#define _HELPERS_H_

template<typename ExpressionType>
boost::python::list getDesignVariablesWrap(const ExpressionType * et) {
  aslam::backend::DesignVariable::set_t dv;
  et->getDesignVariables(dv);

  aslam::backend::DesignVariable::set_t::iterator it = dv.begin();
  boost::python::list dvlist;
  for (; it != dv.end(); ++it) {
    // \todo this (the null_deleter) could be dangerous if the objects go out of scope while Python has them.
    boost::shared_ptr < aslam::backend::DesignVariable
        > val(*it, sm::null_deleter());
    dvlist.append(val);
  }
  return dvlist;
}

#endif /* _HELPERS_H_ */
