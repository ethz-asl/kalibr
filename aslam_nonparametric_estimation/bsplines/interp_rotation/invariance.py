import roslib
roslib.load_manifest('bsplines')
roslib.load_manifest('sm_python')
import bsplines as asp
import numpy as np
from numpy import dot
import pylab as pl
import math
import sm as asrl

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
    ti = math.floor(t)
    rval = 0
    for i in range(0,4):
        rval += bs.eval(t + i)
    return rval
T = np.arange(0,8.0,0.01)
val2 = np.array( [ evalCum(bs,t) for t in T ] )
#pl.plot(T+3,val2)


# It works.
if False:
    T = np.arange(bs.t_min(),bs.t_max(),(bs.t_max() - bs.t_min())/2000)
    val4 = np.array( [ bs.eval(t) for t in T ] )
    val5 = np.array( [ cumForm(bs,t) for t in T ] )
    pl.figure(2)
    pl.clf()
    pl.plot(T,val4,'b-',linewidth=4)
    pl.plot(T,val5,'r--',linewidth=3)

def qeps(q):
    return q[0:3];

def qeta(q):
    return q[3];

def qlog(q):
    #return asrl.quat2AxisAngle(q)
    eta = qeta(q)
    return (2*math.acos(eta)/math.sqrt(1-eta*eta))*qeps(q)

def qexp(a):
    phi = np.linalg.norm(a)
    if phi < 1e-10:
        return np.array([0,0,0,1])
    a = a/phi
    return np.hstack([sin(phi*0.5)*a,cos(phi*0.5)])
    
    #return asrl.axisAngle2quat(a)

def qdot(q1,q2):
    return dot(asrl.quatPlus(q1),q2)

def qinv(q):
    return asrl.quatInv(q)

def clog(C):
    return asrl.r2AxisAngle(C)

def cexp(a):
    return asrl.axisAngle2r(a)


##################
# Clearly the cumulative form is not the integral.
# Try to reproduce the curve in cumulative form.
bs = asp.BSpline(6)
bs.initConstantSpline(0,12,12,np.array([0]))

c = bs.coefficients()
#c = np.random.random(c.shape)
k = bs.knots()
k = np.sort(np.random.random(k.shape)) * 12.0
bs.setKnotVectorAndCoefficients(k,c)

qleft = np.random.random([4,1])
qleft /= np.linalg.norm(qleft)
qright = np.random.random([4,1])
qright /= np.linalg.norm(qright)



# The quat coefficients
qc = np.random.random([4,c.shape[1]])
for i in range(0,c.shape[1]):
    qc[:,i] = qc[:,i]/np.linalg.norm(qc[:,i])

qct = qc.copy()
for i in range(0,c.shape[1]):
    qct[:,i] = qdot(qleft, qdot(qct[:,i],qright))[:,0]


def cumQuat(bs,t,qc):
    si = bs.segmentIndex(t)
    cidx = bs.localCoefficientVectorIndices(t)
    Mi = bs.Mi(si)
    c  = bs.coefficients()
    dc = c[0,1:] - c[0,0:-1]    
    u = bs.u(t,0)
    bi = np.dot(u.T,Mi)

    ci = bs.localCoefficientVectorIndices(t)
    qci = qc[:,ci]
    rval = qci[:,0]
    #print bi
    #print sum(bi)
    #print ci
    for i in range(0,len(ci) - 1):
        dqi = qdot(qinv(qci[:,i]),qci[:,i+1])
        ldqi = qlog(dqi)
        tildeB = np.sum(bi[i+1:])
        rval= qdot(rval, qexp(ldqi * tildeB))
        

    return rval


def cumQuat2(bs,t,qc):
    si = bs.segmentIndex(t)
    cidx = bs.localCoefficientVectorIndices(t)
    Mi = bs.Mi(si)
    c  = bs.coefficients()
    dc = c[0,1:] - c[0,0:-1]    
    u = bs.u(t,0)
    bi = np.dot(u.T,Mi)

    ci = bs.localCoefficientVectorIndices(t)
    qci = qc[:,ci]
    rval = qci[:,0]

    Q = np.zeros([4,len(ci)])
    
    Q[:,0] = qci[:,0]
    #print bi
    #print sum(bi)
    #print ci
    for i in range(0,len(ci) - 1):
        dqi = qdot(qinv(qci[:,i]),qci[:,i+1])
        ldqi = qlog(dqi)
        tildeB = np.sum(bi[i+1:])
        qq = qexp(ldqi * tildeB)
        rval= qdot(rval, qq)
        Q[:,i+1] = qq

    rval = np.eye(4)
    for i in range(len(ci) - 1,0,-1):
        rval = np.dot(rval, asrl.quatPlus(qinv(Q[:,i])))
    rval = qinv(np.dot(rval,qinv(Q[:,0])))

    return rval






if True:
    T = np.arange(bs.t_min()+1e-6,bs.t_max()-1e-6,(bs.t_max() - bs.t_min())/200)    
    qval = np.array( [ qdot(qleft,qdot(cumQuat2(bs,t,qc),qright))[:,0] for t in T ] )
    qvalt = np.array( [ cumQuat2(bs,t,qct) for t in T ] )

    
    nm = np.sum(qval * qval,1)
    pl.figure(3)
    pl.clf()
    pl.plot(T,qval,linewidth=4)
    pl.plot(T,qvalt,linewidth=3,ls='--')
    #pl.plot(T,nm,linewidth=4)
    #pl.plot(k[3:-3],qc.T,'.k',markersize=4)

