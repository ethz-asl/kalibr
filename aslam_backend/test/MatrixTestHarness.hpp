#ifndef _MATRIXTESTHARNESS_H_
#define _MATRIXTESTHARNESS_H_

#include <aslam/backend/Matrix.hpp>

class MatrixTestHarness {
public:
  MatrixTestHarness(aslam::backend::Matrix* M);
  virtual ~MatrixTestHarness();

  void testDense();

  void testRightMultiply();

  void testLeftMultiply();

  void testAll();


  aslam::backend::Matrix* matrix;
};


#endif /* _MATRIXTESTHARNESS_H_ */
