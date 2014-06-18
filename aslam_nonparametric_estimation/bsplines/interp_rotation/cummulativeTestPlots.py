import roslib
roslib.load_manifest('bsplines')
roslib.load_manifest('sm_python')
import bsplines as asp
import numpy as np
import pylab as pl

#pl.figure(1)
bs = asp.BSpline(4)
bs.initConstantSpline(0,12,12,np.array([0]))
bs.setCoefficientMatrix(np.array([[0,0,0,0,0,0,0,1,0,0,0,0,0,0,0]]))
T = np.arange(0,12.0,0.01)
val = np.array( [ bs.eval(t) for t in T ] )

# This is the regular basis function.
#pl.plot(T,val)

def evalCum(bs,t):
    rval = 0
    for i in range(0,4):
        rval += bs.eval(t + i)
    return rval
T8 = np.arange(0,8.0,0.01)
val2 = np.array( [ evalCum(bs,t) for t in T8 ] )
#pl.plot(T8+3,val2)


def evalLocalBi(bs, t, i):
    return bs.getLocalBi(t)[i - bs.segmentIndex(t)] if i >= bs.segmentIndex(t) and i <= bs.segmentIndex(t) + 3 else 0

def getLocalCumulativeBi(bs, t, i):
    return bs.getLocalCumulativeBi(t)[i - bs.segmentIndex(t)] if i >= bs.segmentIndex(t) and i <= bs.segmentIndex(t) + 3 else 1 if i < bs.segmentIndex(t) else 0


c = -1

for i in range(0, 3) :
    print i
#    val = np.array( [ getLocalCumulativeBi(bs, t, i) for t in T ] )
#    pl.plot(T,val)
#    val = np.array( [ getLocalCumulativeBi(bs, t+c, i+c) if (t + c) < 8 and (t+c >0) else 0 for t in T ] )
#    pl.plot(T,val)
#    val = np.array( [ evalLocalBi(bs, t, i) for t in T ] )
#    pl.plot(T,val)
#    val = np.array( [ evalLocalBi(bs, t+c, i+c) if (t + c) < 8 and (t+c >0) else 0 for t in T ] )
#    pl.plot(T,val)

    val = np.array( [ bs.getBiFunction(t)(i) + 0.1 for t in T ] )
    pl.plot(T,val)
    val = np.array( [ bs.getCumulativeBiFunction(t)(i) + 0.1 for t in T ] )
    pl.plot(T,val)


