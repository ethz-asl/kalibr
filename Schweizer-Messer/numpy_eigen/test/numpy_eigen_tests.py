#!/usr/bin/env python
import roslib; roslib.load_manifest('numpy_eigen'); roslib.load_manifest('rostest'); 
import numpy_eigen
import numpy_eigen.test as npe
import numpy

import sys
# http://docs.python.org/library/unittest.html#test-cases
import unittest



class TestEigen(unittest.TestCase):
    def assertMatrixClose(self,numpyM,eigenM,testType):
        self.assertEqual(numpyM.size,eigenM.size)
        if eigenM.ndim == 1:
            # The eigen conversion code will take a 1xN or Nx1 and turn
            # it into a single dimension matrix.
            if numpyM.ndim == 1:
                # The original matrix was 1d...compare the 1d types.
                self.assertEqual(numpyM.shape[0],eigenM.shape[0], testType)
                self.assertTrue(numpy.max(numpy.abs(numpyM - eigenM)) < 1e-10, testType)
            elif numpyM.ndim == 2:
                # The original matrix was 2d...compare the 1d dimension
                # with the eigen matrix
                if numpyM.shape[0] == 1:
                    # Row vector
                    self.assertEqual(numpyM.shape[1],eigenM.shape[0], testType)
                    if eigenM.shape[0] > 0:
                        self.assertTrue(numpy.max(numpy.abs(numpyM[0,:] - eigenM)) < 1e-10, testType)
                elif numpyM.shape[1] == 1:
                    # column vector
                    self.assertEqual(numpyM.shape[0],eigenM.shape[0], testType)
                    if eigenM.shape[0] > 0:
                        self.assertTrue(numpy.max(numpy.abs(numpyM[:,0] - eigenM)) < 1e-10, testType)
                else:
                    self.fail('%s: The output matrix is a vector but none of the input matrix dimensions are 1: %s' % (testType,numpyM.shape))
            else:
                self.fail('%s: Unexpected number of dimensions in the numpy input matrix: %d' % (testType,numpyM.ndim))
        elif eigenM.ndim == 2:
            self.assertEqual(numpyM.shape[0],eigenM.shape[0], testType)
            self.assertEqual(numpyM.shape[1],eigenM.shape[1], testType)
            if numpyM.shape[0] > 0 and numpyM.shape[1] > 0:
                self.assertTrue(numpy.max(numpy.abs(numpyM - eigenM)) < 1e-10, testType)
        else:
            self.fail('%s: Unexpected number of dimensions in the numpy output matrix: %d' % (testType, eigenM.ndim))
        
    def matrixTests(self,t,i,j):
        row_limit = 50
        col_limit = 50
        rows_dim_is_dynamic = (i == 'D')
        cols_dim_is_dynamic = (j == 'D')
        ii = i;
        jj = j;
        if not rows_dim_is_dynamic:
            ii = int(ii)
        if not cols_dim_is_dynamic:
            jj = int(jj)
        fname = 'test_%s_%s_%s' % (t,i,j)
        
        for R in range(0,row_limit):
            for C in range(0,col_limit):
                testType = 'Testing %s with input array[%d,%d]' % (fname, R, C)
                try:
                    # Create a random matrix.
                    numpyM = numpy.random.random([R,C])
                    # Try to pass it in to the pass-through function
                    eigenM = npe.__dict__[fname](numpyM)
                    # There was no error...check that this was okay.
                    self.assertTrue(rows_dim_is_dynamic or R == ii, testType)
                    self.assertTrue(cols_dim_is_dynamic or C == jj, testType)
                    
                    # Check that the matrices are the same.
                    self.assertMatrixClose(numpyM,eigenM, testType)
                except TypeError as inst:
                    # There was a type error. Check that this was expected.
                    self.assertFalse( (rows_dim_is_dynamic or R == ii) and (cols_dim_is_dynamic or C == jj), testType)
                
                try:
                    # Create a random matrix and take the transpose.
                    numpyM = numpy.random.random([C,R]).T
                    # Try to pass it in to the pass-through function
                    eigenM = npe.__dict__[fname](numpyM)
                    # There was no error...check that this was okay.
                    self.assertTrue(rows_dim_is_dynamic or R == ii, testType)
                    self.assertTrue(cols_dim_is_dynamic or C == jj, testType)
                    
                    # Check that the matrices are the same.
                    self.assertMatrixClose(numpyM,eigenM, testType)
                except TypeError as inst:
                    # There was a type error. Check that this was expected.
                    self.assertFalse( (rows_dim_is_dynamic or R == ii) and (cols_dim_is_dynamic or C == jj), testType)
                    
                    
    def vectorTests(self,t,i,j):
        x = 1
        # um...
        
    def test_eigen(self):
        T = ['double']
        #N = ('01','02','03','04','05','06','07','08','09','10','11','12','13','14','15','16','D')
        N = ('1','2','3','4','5','6','D')
        #N = (1,2,3,4,'dynamic')
        for t in T:
            for i in N:
                for j in N:
                    self.matrixTests(t,i,j)
        



if __name__ == '__main__':
    import rostest
    rostest.rosrun('numpy_eigen', 'test_eigen', TestEigen)
