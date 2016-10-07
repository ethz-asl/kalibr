#!/usr/bin/env python
import asrl_splines
import numpy
import scipy.interpolate.fitpack as fp
import sys
import unittest


class TestSplinesVsFitpack(unittest.TestCase):
    def test_new_uniform_cubic1(self):
        # create the knot and control point vectors
        knots = numpy.array([-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
        cp    = numpy.array([0.0, 10.0, 2.0, 30.0, 4.0, 50.0])
        cpa    = numpy.array([cp])
        
        aspl = asrl_splines.UniformCubicBSpline(cpa,1.0,knots[3]);
        fspl = (knots,cp,3)
        
        for i in numpy.arange(2.0,4.0,0.1):
            print "Eval at %f\n" % (i)
            f = fp.spalde(float(i),fspl)
            for j in range(0,3):
                a = aspl.evalD(i,j)
                assert abs(a - f[j]) < 1e-10, "spline (D%d) evaluated at %f (%f != %f) was not right" % (j,float(i),a,f[j])

    def test_new_uniform_cubic2(self):
        # create the knot and control point vectors
        cardinalknots = numpy.array([-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
        cp    = numpy.array([0.0, 10.0, 2.0, 30.0, 4.0, 50.0])
        cpa    = numpy.array([cp])

        for dt in numpy.arange(0.1,2.0,0.1):
            knots = cardinalknots * dt;
        
            aspl = asrl_splines.UniformCubicBSpline(cpa,dt,knots[3]);
            fspl = (knots,cp,3)
        
            for i in numpy.arange(2.0*dt,4.0*dt,0.1*dt):
                print "Eval at %f\n" % (i)
                f = fp.spalde(float(i),fspl)
                for j in range(0,3):
                    a = aspl.evalD(i,j)
                    assert abs(a - f[j]) < 1e-10, "spline (D%d) evaluated at %f (%f != %f) was not right" % (j,float(i),a,f[j])


    def test_new_uniform_cubicIntegral(self):
        # create the knot and control point vectors
        cardinalknots = numpy.array([-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
        cp    = numpy.array([0.0, 10.0, 2.0, 30.0, 4.0, 50.0])
        cpa    = numpy.array([cp])
        
        for dt in numpy.arange(0.1,2.0,0.1): 
            knots = cardinalknots * dt;

        
            aspl = asrl_splines.UniformCubicBSpline(cpa,dt,knots[3]);
            fspl = (knots,cp,3)
        
            for a in numpy.arange(2.0*dt,3.9*dt,0.1*dt):
                for i in numpy.arange(2.1*dt,4.0*dt,0.1*dt):
                    print "Eval at %f\n" % (i)
                    f = fp.splint(a,float(i),fspl)
                    b = aspl.evalI(a,i)
                    assert abs(b - f) < 1e-10, "spline integral evaluated on [%f,%f] (%f != %f) was not right" % (a,i,float(b),f)




            

if __name__ == '__main__':
    import rostest
    rostest.rosrun('splines', 'test_vs_fitpack', TestSplinesVsFitpack)

