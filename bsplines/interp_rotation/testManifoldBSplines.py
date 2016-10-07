from threeManifoldVisual import Manifold
import numpy
import math
from diffManifolds import UnitQuaternionManifold, EuclideanSpace 
from visual import color
from bsplines import BSpline
import diffManifoldBSplines

geometry = EuclideanDiffManifold()
pointLength = geometry.getIdentity().shape[0]


splineOrder = 4
order = splineOrder - 1
time_max = 20
time_min = 0

numberOfSegments = 80 
ebs = diffManifoldBSplines.createExpBSpline(geometry, splineOrder, time_min, time_max, numberOfSegments)
numPoints = ebs.numVvCoefficients()

p = numpy.array((0, ))
points = [p] 
for i in range(0, numPoints - 1):
    v = numpy.random.random(1)- 0.5
    v = (0.1, )
    if(v[0]< 0) :
        v = v * -1
    p = geometry.exp(p, v * 1) 
    points.append(p)

points = numpy.array(points)

ebs.setControlVertices(points)

espline = [ ebs.eval(t) for t in numpy.arange(time_min, time_max, 0.1)]
print numpy.array(espline).transpose();

bs = ebs.getBSpline();
bs.setCoefficientMatrix(points.transpose())
spline = [ bs.eval (t) for t in numpy.arange(time_min, time_max, 0.1)]
print numpy.array(spline).transpose();
