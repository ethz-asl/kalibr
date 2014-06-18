from threeManifoldVisual import Manifold
import os
import numpy
import math
from diffManifolds import UnitQuaternionManifold, EuclideanSpace 
from visual import color
from bsplines import BSpline, UnitQuaternionBSpline, EuclideanBSpline
import diffManifoldBSplines


randomSamples = False

direction = numpy.array( (0.5, 0.5, 0) )

splineOrder = 4
order = splineOrder - 1
time_max = 3
time_min = 0

numberOfSegments = 80 if randomSamples else 4


geometry = UnitQuaternionManifold()
#geometry = EuclideanSpace()

pointLength = geometry.getIdentity().shape[0]

def createExpBSpline(geometry, splineOrder, time_min, time_max, numberOfSegments):
    if isinstance(geometry, UnitQuaternionManifold):
        ebs = UnitQuaternionBSpline(splineOrder)
        ebs.initConstantSpline(time_min, time_max, numberOfSegments, numpy.array((0, 0, 0, 1)));
    else:
        ebs = diffManifoldBSplines.createExpBSpline(geometry, splineOrder, time_min, time_max, numberOfSegments)
        #ebs.setControlVertices = lambda points, old=ebs.setControlVertices : old(points.transpose())
    return ebs


def calculateGeodetic(start, direction, length, steps):
    ret = [start]
    
    diff = geometry.exp(geometry.getIdentity(), direction * (length / steps / numpy.linalg.norm(direction)))
    
    for __i in range(0, steps) :
        start = geometry.product(start, diff)
        ret.append(start)
    return numpy.array(ret);

def createPointWithGeodetics(point, directions, color):
    q = numpy.array(point)
    manifold.addPoint(q, color).setRadius(0.05)
    for direction in directions: 
        geo = calculateGeodetic(q, numpy.array(direction), 1 * math.pi, 32)
        manifold.addCurve(geo, color).setRadius(0.001)


manifold = Manifold(geometry.getIdentity(), geometry)

directions = ((1, 0, 0),(0, 1, 0),(0, 0, 1))
if True:
    if(pointLength == 4):
        createPointWithGeodetics((1, 0, 0, 0), directions, color.green)
        createPointWithGeodetics((-1, 0, 0, 0), directions, color.green)
        createPointWithGeodetics((0, 1, 0, 0), directions, color = color.blue)
        createPointWithGeodetics((0, -1, 0, 0), directions, color = color.blue)
        createPointWithGeodetics((0, 0, 1, 0), directions, color = color.red)
        createPointWithGeodetics((0, 0, -1, 0), directions, color = color.red)
        createPointWithGeodetics((0, 0, 0, 1), directions, color = color.yellow)
        createPointWithGeodetics((0, 0, 0, -1), directions, color = color.yellow)
    else:
        createPointWithGeodetics((1, 0, 0), directions, color.green)
        createPointWithGeodetics((-1, 0, 0), directions, color.green)
        createPointWithGeodetics((0, 1, 0), directions, color = color.blue)
        createPointWithGeodetics((0, -1, 0), directions, color = color.blue)
        createPointWithGeodetics((0, 0, 1), directions, color = color.red)
        createPointWithGeodetics((0, 0, -1), directions, color = color.red)

ebs = createExpBSpline(geometry, splineOrder, time_min, time_max, numberOfSegments)

numPoints = ebs.numVvCoefficients()

p = numpy.array(geometry.getIdentity())
points = [p] * (splineOrder - 1) 
for i in range(0, numberOfSegments):
    v = numpy.random.random(3)- 0.5
    if(v[0]< 0) :
        v = v * -1
    p = geometry.exp(p, v * 1) 
    points.append(p)

points = numpy.array(points)

ebs.setControlVertices(points)
#manifold.addCurve(points[order -1:], color.red).setRadius(0.005)

spline = [ ebs.eval (t) for t in numpy.arange(time_min, time_max, 0.01)]

manifold.addCurve(spline, color.magenta).setRadius(0.005)

samples = []
sampleNumber = numberOfSegments - 5 if randomSamples else 4
delta = float(time_max - time_min) / (sampleNumber - 1)
T = numpy.arange(time_min, time_max + 1e-9, delta)

#print T[sampleNumber -1]
p = geometry.getIdentity()
for t in T :
    if randomSamples :
        v = numpy.random.randn(3) / 30
        samples.append(geometry.exp(ebs.eval(t), v))
    else :
        samples.append(p)
        print "sample ", t, " -> ", p
        v = direction
        p = geometry.exp(p, v)

manifold.addCurve(samples, color.orange).setRadius(0.005)

fitebs = createExpBSpline(geometry, splineOrder, time_min, time_max, numberOfSegments)

fitSelf = False
if fitSelf : 
    sflat = numpy.array(samples).flatten()
    fitControlVerticesNumber = fitebs.numVvCoefficeients()
    A = numpy.zeros((pointLength * len(T), pointLength * fitControlVerticesNumber))
    bs = fitebs.getBSpline()
    for iT in range(0, len(T)):
        t = T[iT]
        ib = iT * pointLength
        Bis = bs.getLocalBi(t);
        ci = bs.localCoefficientVectorIndices(t)
        for jj in range(0, len(ci)):
            j = ci[jj]
            bj = Bis[jj]
            jb = j * pointLength
            for i in range(0, pointLength):
                A[ib + i][jb + i] = bj
    x = numpy.linalg.lstsq(A, sflat, 1e-6)[0]
    fitCP = x.reshape(fitControlVerticesNumber, pointLength);
else :
    bs = BSpline(splineOrder)
    bs.initSplineSparse(numpy.array(T), numpy.array(samples).transpose(), numberOfSegments, 1e-5)
    #print bs.splineOrder()
    fitControlVerticesNumber = fitebs.numVvCoefficients()
    fitCP = bs.coefficients().transpose()

if isinstance(geometry, UnitQuaternionManifold):
    for i in range(0, fitControlVerticesNumber):
        norm = numpy.linalg.norm(fitCP[i])
        if(norm < 1e-6) :
            fitCP[i] = fitCP[i-1] if i > 0 else fitCP[i+1]/numpy.linalg.norm(fitCP[i+1])
        else:
            fitCP[i] = fitCP[i] / norm

print fitCP

fitebs.setControlVertices(fitCP)
fitspline = [ fitebs.eval(t) for t in numpy.arange(time_min, time_max+ 1e-9, 0.01)]

#manifold.addCurve(fitCP, color.red).setRadius(0.005)
manifold.addCurve(fitspline, color.cyan).setRadius(0.005)

manifold.setZeroPointEnabled(True)
manifold.startInteractionThread();


ShowAngularDerivative = True
class Pointer :
    def __init__(self):
        self._curve = manifold.addCurve([ numpy.array((0, 1, 0, 0)), numpy.array((0, 0, 0, 1)) ], color.red)
        self._curve.setRadius(0.01)
        
    def updatePointer(self, p, tpos):
        if isinstance(fitebs, UnitQuaternionBSpline):
            if ShowAngularDerivative :
                v = fitebs.getEvaluatorAt(tpos).evalAngularVelocity()
                print v
            else:
                v = fitebs.evalD(tpos, 1)
                v = geometry.product(geometry.inv(p), v) # pull the vector back to identity
            v = geometry.exp(p, v[0 : 3]) # go in its direction starting from p 
            self._curve.setPos([ p, v ])
        else:
            print fitebs

pointer = Pointer()

def gotoSplinePos(tpos):
    p = fitebs.eval(tpos)
    pointer.updatePointer(p, tpos)
    manifold.setCurrentPos(p)
    

gotoSplinePos(time_min)


class TPos:
    tpos=0
    tstep=0.125

    def step(self, direction):
        self.goto(max(time_min, min(time_max, self.tpos + direction * self.tstep)))

    def goto(self, tpos):
        self.tpos = tpos
        gotoSplinePos(tpos)

tp = TPos()


manifold.getKeyHandler()["w"] = lambda : tp.step(1)
manifold.getKeyHandler()["s"] = lambda : tp.step(-1)
manifold.getKeyHandler()["home"] = lambda : tp.goto(time_min)
manifold.getKeyHandler()["end"] = lambda : tp.goto(time_max)
manifold.getKeyHandler()["q"] = lambda : os._exit(0)
