import bsplines as asp
import numpy as np
import pylab as pl
from quaternions import qdot, qlog, qexp, qinv

#pl.figure(1)
bs = asp.BSpline(4)
bs.initConstantSpline(0,12,12,np.array([0]))
bs.setCoefficientMatrix(np.array([[0,0,0,0,0,0,0,1,0,0,0,0,0,0,0]]))
T = np.arange(0,12.0,0.01)
val = np.array( [ bs.eval(t) for t in T ] )

# This is the regular basis function.
#pl.plot(T,val)

val3 = np.array( [ bs.evalI(0,t) for t in T ] )
#pl.plot(T,val3,'b:')
def evalCum(bs,t):
    rval = 0
    for i in range(0,4):
        rval += bs.eval(t + i)
    return rval
T = np.arange(0,8.0,0.01)
val2 = np.array( [ evalCum(bs,t) for t in T ] )
#pl.plot(T+3,val2)


# It works.
if False:
    T = np.arange(bs.t_min(),bs.t_max()-1e-14,(bs.t_max() - bs.t_min())/2000)
    val4 = np.array( [ bs.eval(t) for t in T ] )
    val5 = np.array( [ cumForm(bs,t) for t in T ] )
    pl.figure(2)
    pl.clf()
    pl.plot(T,val4,'b-',linewidth=4)
    pl.plot(T,val5,'r--',linewidth=3)


##################
# Clearly the cumulative form is not the integral.
# Try to reproduce the curve in cumulative form.
bs = asp.BSpline(6)
bs.initConstantSpline(0,12,12,np.array([0]))

c = bs.coefficients()
#c = np.random.random(c.shape)
k = bs.knots()
#k = np.sort(np.random.random(k.shape)) * 12.0
#bs.setKnotVectorAndCoefficients(k,c)

# The quat coefficients
qc = np.random.random([4,c.shape[1]])
for i in range(0,c.shape[1]):
    qc[:,i] = qc[:,i]/np.linalg.norm(qc[:,i])
    
    
#qc[:, 5] = qc[:, 4]
#qc[:, 6] = qc[:, 4]
#qc[:, 7] = qc[:, 4]
#qc[:, 8] = qc[:, 4]

def cumQuat(bs,t,qc):
    si = bs.segmentIndex(t)
    Mi = bs.Mi(si)
    u = bs.u(t,0)
    bi = np.dot(u.T,Mi)
    
    cumulativeBi = bs.getCumulativeBiFunction(t);
    
    ci = bs.localCoefficientVectorIndices(t)
    
    qci = qc[:,ci]
    rval = qci[:,0]
    for i in range(0,len(ci) - 1):
        dqi = qdot(qinv(qci[:,i]),qci[:,i+1])
        ldqi = qlog(dqi)
        tildeB = np.sum(bi[i+1:])
        diff = tildeB - cumulativeBi(int(ci[i]+1))
        if abs(diff) > 1e-9 :
            print diff
        rval= qdot(rval, qexp(ldqi * tildeB))
        

    return rval

def cumQuat2(bs,t,qc):
    cumulativeBi = bs.getLocalCumulativeBi(t);
    ci = bs.localCoefficientVectorIndices(t)
    qci = qc[:,ci]
    rval = qci[:,0]
    for i in range(0,len(ci) - 1):
        dqi = qdot(qinv(qci[:,i]),qci[:,i+1])
        ldqi = qlog(dqi)
        rval= qdot(rval, qexp(ldqi * cumulativeBi[i+1]))
        
    return rval

from exponentialBSplines import ExponentialBSpline
from exponentialDiffManifold import UnitQuaternionManifold

ebs = ExponentialBSpline(UnitQuaternionManifold(), bs)
ebs.setControlVertices(qc.transpose())

def cumQuat3(bs,t,qc):
    return ebs.evaluate(t)

if True:
    T = np.arange(bs.t_min(),bs.t_max()-1e-6,(bs.t_max() - bs.t_min())/200)
    
    qval = np.array( [ cumQuat3(bs,t,qc) for t in T ] ).T
    
    nm = np.sum(qval * qval,0)
    for i in range(len(nm)):
        if nm[i] - 1 > 1E-9 :
            print qval[i]
    pl.figure(3)
    pl.clf()
    pl.plot(T,qval[0,:],'r-',linewidth=4)
    pl.plot(T,qval[1,:],'g-',linewidth=4)
    pl.plot(T,qval[2,:],'b-',linewidth=4)
    pl.plot(T,qval[3,:],'k-',linewidth=4)
    pl.plot(T,nm,linewidth=4)
    pl.plot(k[3:-3],qc[0,:],'.r',markersize=10)
    pl.plot(k[3:-3],qc[1,:],'.g',markersize=10)
    pl.plot(k[3:-3],qc[2,:],'.b',markersize=10)
    pl.plot(k[3:-3],qc[3,:],'.k',markersize=10)


print('-- end of line --')
