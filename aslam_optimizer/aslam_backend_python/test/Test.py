#!/usr/bin/env python
import aslam_backend as ab
import numpy as np

import unittest

class TestJacobianContainerSparse(unittest.TestCase):
  def test_add(self):
    point = ab.Point2d(np.array([1,1]))
    point.setBlockIndex(0)
    point.setActive(True)
    jc = ab.JacobianContainerSparse(2)
    self.assertEqual(jc.rows(), 2)
    J = np.array([[1,1], [1,1]])
    jc.add(point, J) # With design variable
    self.assertTrue(jc.isFinite(point))
    self.assertTrue( (jc.asDenseMatrix() == J).all() )
    jc2 = ab.JacobianContainerSparse(2)
    jc2.add(point, J) # With design variable
    jc.add(jc2) # Without chain rule
    self.assertTrue( (jc.asDenseMatrix() == 2.*J).all() )
    jc2 = ab.JacobianContainerSparse(2)
    jc2.add(point, J) # With design variable
    jc.add(jc2, np.eye(2,2)) # With chain rule
    self.assertTrue( (jc.asDenseMatrix() == 3.*J).all() )
    
class TestJacobianContainerDense(unittest.TestCase):
  def test_add(self):
    point = ab.Point2d(np.array([1,1]))
    point.setBlockIndex(0)
    point.setColumnBase(0)
    point.setActive(True)
    jc = ab.JacobianContainerDense(2, point.minimalDimensions())
    self.assertEqual(jc.rows(), 2)
    J = np.array([[1,1], [1,1]])
    jc.add(point, J) # With design variable
    self.assertTrue(jc.isFinite(point))
    self.assertTrue( (jc.asDenseMatrix() == J).all() )

class TestRprop(unittest.TestCase):
    def test_simple_optimization(self):
        options = ab.OptimizerOptionsRprop()
        options.maxIterations = 500;
        options.numThreads = 1;
        options.convergenceGradientNorm = 1e-6
        options.method = ab.RpropMethod.RPROP_PLUS
        optimizer = ab.OptimizerRprop(options)
        problem = ab.OptimizationProblem()
        
        D = 2
        E = 3
        
        # Add some design variables.
        p2ds = [];
        for d in range(0, D):
          point = ab.Point2d(np.array([d, d]))
          p2ds.append(point);
          problem.addDesignVariable(point);
          point.setBlockIndex(d);
          point.setActive(True);
        
        # Add some error terms.
        errors = [];
        for e in range(0, E):
          for d in range(0, D):
            grad = np.array( [d+1, e+1] );
            err = ab.TestNonSquaredError(p2ds[d], grad);
            err._p = 1.0;
            errors.append(err);
            problem.addScalarNonSquaredErrorTerm(err);
            
        # Now let's optimize.
        optimizer.setProblem(problem);
        optimizer.checkProblemSetup();
        optimizer.optimize();
    
        self.assertLessEqual(optimizer.status.gradientNorm, 1e-3);
        
class TestMetropolisHastings(unittest.TestCase):
    def test_simple_sampling_problem(self):
        '''Sample from a twodimensional Gaussian distribution N(0, I)
        '''
      
        options = ab.SamplerMetropolisHastingsOptions()
        options.transitionKernelSigma = 0.1;
        sampler = ab.SamplerMetropolisHastings(options)
        negLogDensity = ab.OptimizationProblem()

        # Add a scalar design variables.
        point = ab.Point2d(np.array([0, 0]))
        point.setBlockIndex(0);
        point.setActive(True);
        negLogDensity.addDesignVariable(point);

        # Add the error term.
        grad = np.array( [-1., -1.] );
        err = ab.TestNonSquaredError(point, grad);
        err._p = 0.0; # set mean
        negLogDensity.addScalarNonSquaredErrorTerm(err);

        # Now let's sample.
        sampler.setNegativeLogDensity(negLogDensity);
        sampler.checkNegativeLogDensitySetup();
        sampler.run(10000);
        
if __name__ == '__main__':
    import rostest
    rostest.rosrun('aslam_backend_python', 'Rprop', TestRprop)
    rostest.rosrun('aslam_backend_python', 'MetropolisHastings', TestMetropolisHastings)