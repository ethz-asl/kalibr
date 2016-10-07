#!/usr/bin/env python
import aslam_backend
import bsplines
import aslam_splines
import numpy
import scipy.interpolate.fitpack as fp
import scipy.integrate as integrate

import unittest

def createUniformKnotBSpline(order,segments,dim,knotSpacing=1.0):
    aspl = aslam_splines.OptEuclideanBSpline(order, dim)
    aspl.initConstantUniformSpline(0, segments * knotSpacing, segments, numpy.zeros((dim, 1)))
    kc = aspl.getNumControlVertices();
    cp = numpy.random.random([dim,kc])
    aspl.setControlVertices(cp)
    return (aspl,(aspl.getKnotsVector(),cp,order-1))

def createUniformKnotOldBSpline(order,segments,dim,knotSpacing=1.0):
    aspl = bsplines.BSpline(order)
    kr = aspl.numKnotsRequired(segments)
    kc = aspl.numCoefficientsRequired(segments);
    knots = numpy.linspace(-order,kr - 1 - order, kr)*knotSpacing
    cp = numpy.random.random([dim,kc])
    aspl.setKnotVectorAndCoefficients(knots, cp)
    return aspl

class TestQuadraticIntegralError(unittest.TestCase):
    def test_construction(self):
        splineOrder = 4;
        spline = createUniformKnotOldBSpline(splineOrder, 6, 3)
        sdv = aslam_splines.EuclideanBSplineDesignVariable(spline)

        tMin = spline.t_min();
        tMax = spline.t_max();
        sqrtW = numpy.matrix(numpy.random.random((3, 3)))
        W = sqrtW.T * sqrtW;
        for derivativeOrder in range(0, splineOrder - 1):
            p = aslam_backend.OptimizationProblem()
            for i in range(0, sdv.numDesignVariables()) : p.addDesignVariable(sdv.designVariable(i))
            aslam_splines.addQuadraticIntegralEuclideanExpressionErrorTermsToProblem(p, tMin, tMax, 100, lambda(time) : sdv.toEuclideanExpression(time, derivativeOrder), sqrtW)
            E = 0;
            for i in range(0, p.numErrorTerms()):
                E += p.errorTerm(i).evaluateError()
    
            evalQF = lambda t: (numpy.matrix(spline.evalD(t, derivativeOrder)) * W * numpy.matrix(spline.evalD(t, derivativeOrder)).T)[0, 0];
            Epy = integrate.quad(evalQF, tMin, tMax)[0]
            
            self.assertAlmostEqual(E, Epy, 3); 

if __name__ == '__main__':
    import rostest
    rostest.rosrun('splines', 'QuadraticIntegralError', TestQuadraticIntegralError)
