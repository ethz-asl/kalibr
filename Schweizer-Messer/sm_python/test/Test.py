#!/usr/bin/env python
import os
import sm
import numpy as np

import unittest

from nose import SkipTest

class TestMatrixArchive(unittest.TestCase):
  def test_saveLoad(self):
    if sm.numpy_eigen.IsBroken : self.skipTest("numpy_eigen is broken (see #137)");
    ma = sm.MatrixArchive()
    ma.setMatrix("testM", np.array([[0, 0], [1, 1], [2, 2]]))
    testSValue = "testStringValue"
    ma.setString("testS", testSValue)
    self.assertTrue("testM" in ma.getNameList())
    self.assertEqual(ma.getMatrix("testM").size, 6)
    self.assertEqual(ma.getString("testS"), testSValue)
    
    testAma = "TestMatrixArchive-test.ama"
    ma.save(testAma)
    
    ma = sm.MatrixArchive()
    self.assertFalse("testM" in ma.getNameList())
    ma.load(testAma)
    self.assertTrue("testM" in ma.getNameList())
    self.assertEqual(ma.getMatrix("testM").size, 6)
    self.assertEqual(ma.getString("testS"), testSValue)
    os.unlink(testAma)

class TestNsecTime(unittest.TestCase):
  def test_secToNsec(self):
    self.assertEqual(sm.secToNsec(1), 1.0e9)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('sm_python', 'TestMatrixArchive', TestMatrixArchive)
    rostest.rosrun('sm_python', 'TestNsecTime', TestNsecTime)
