from scipy import integrate
from pylab import arange, array, show, plot, axis
import bsplines

splineOrder = 4;
s = bsplines.EuclideanBSpline(splineOrder, 1)

maxTime = 9
minTime = 0

pos = maxTime;
lamb = 0.00001
goal = 1.0

knotGenerator = s.initConstantUniformSplineWithKnotDelta(minTime, maxTime, 1, array([0]));

print "Knots:" + str(s.getKnotsVector());

def plotSpline(s, showIt = False):
    t = arange(s.getMinTime(), s.getMaxTime(), 0.01)
    v = array([ s.eval(tt) for tt in t])
    plot(t, v)
    if showIt : show()

def getAcceleration(s):
    return integrate.quad(lambda t: s.evalD(t, 2) * (s.evalD(t, 2)), s.getMinTime(), s.getMaxTime())[0]

plotSpline(s)
print "AccelerationIntegral:\n", getAcceleration(s)

def calcFit():
    goalDist = s.eval(pos)[0] - goal
    a = getAcceleration(s)
    return goalDist*goalDist - lamb * a


s.fitSpline(array([pos]), array([[goal]]), lamb, 0, array([]))

plotSpline(s)
print "Value:\n", s.eval(pos);
print "AccelerationIntegral:\n", getAcceleration(s)
print "Vertices:\n", s.getControlVertices();
print "Fit:", calcFit()


s.extendAndFitSpline(knotGenerator, array([pos - 1, maxTime * 2]), array([[0, goal]]), lamb, 10)

plotSpline(s)
print "MaxTime:\n", s.getMaxTime();
print "Value:\n", s.eval(pos);
print "AccelerationIntegral:\n", getAcceleration(s)
print "Vertices:\n", s.getControlVertices();
print "Fit:", calcFit()

if False:
    c = s.getControlVertices()
    c[maxTime - 1 + 3]+=0.8;
    c[maxTime - 2 + 3]-=0.2;
    #c[maxTime - 5 + 3]+=0.11404781;
    s.setControlVertices(c)
    plotSpline(s)
    print "Value:\n", s.eval(pos);
    print "AccelerationIntegral:\n", getAcceleration(s)
    print "Vertices:\n", s.getControlVertices();


if True:
    s2 = bsplines.EuclideanBSpline(4, 1)
    n = 1;
    ts=[]; 
    z = max(0, pos - 4);
    ts.extend([ z * float(x)/ n for x in range(0, n + 1, 1) ])
    ts.extend([pos for x in range(0, n + 1, 1)])
    z = maxTime - min(maxTime, pos + 4)
    ts.extend([ maxTime - z + z* float(x) / n  for x in range(0, n + 1, 1) ])
    values = [ 0 if x != pos else goal for x in ts ]
    print ts , values; 
    
    s2.initUniformSpline(array(ts), array([values]), maxTime, lamb * n)

    print "Value:\n", s2.eval(pos);
    print "AccelerationIntegral:\n", getAcceleration(s)
    print "Vertices:\n", s2.getControlVertices();
    
    plotSpline(s2)

axis([0, maxTime * 2, -0.5, 1.5])
show()

print "finished"
