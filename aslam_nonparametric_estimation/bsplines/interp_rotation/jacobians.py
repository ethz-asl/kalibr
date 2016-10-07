import roslib
roslib.load_manifest('splines')
import asrl_splines as asp
import numpy as np
import pylab as pl
import math
import asrl
import numdifftools as nd
from numpy import dot, eye
from math import sin
from math import cos

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

def dotn(*mats):
    if len(mats) == 2:
        return np.dot(mats[0],mats[1])
    else:
        return np.dot(mats[0],dotn(*mats[1:]))

def qplus(q):
    return asrl.quatPlus(q)

def qoplus(q):
    return asrl.quatOPlus(q)

def qeps(q):
    return q[0:3];

def qeta(q):
    return q[3];

def qlog(q):
    #return asrl.quat2AxisAngle(q)
    eta = qeta(q)
    return (2*math.acos(eta)/math.sqrt(1-eta*eta))*qeps(q)

def qexp(a):
    if len(a.shape) == 2:
        a = a[:,0]
    phi = np.linalg.norm(a)
    if phi < 1e-10:
        return np.array([0,0,0,1])
    a = a/phi
    q = np.zeros(4)
    q[0:3] = sin(phi*0.5)*a;
    q[3] = cos(phi*0.5)
    return q
    
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



def ljac(q):
    lq = qlog(q)
    eta = qeta(q)
    c = 2 * math.acos(eta)
    d = 1 - eta * eta
    L = np.hstack([c * eye(3), (1/d) * ( eta * lq - 2 * qeps(q))])
    return L

def V():
    return np.eye(4,3) * 0.5

def invS(phi):
    p = math.sqrt(dot(phi.T,phi))
    a1 = ( 1 / (p*p) ) * ( 1 - 0.5 * p / math.tan(0.5 * p))
    px = asrl.crossMx(phi)
    return eye(3) + 0.5 * px + a1 * dot(px,px)

def L(q):
    return invS(log(q))

def randomQuat():
	q = np.random.random([4,1]) - 0.5
	q = q / np.linalg.norm(q)
	return q

qim1 = randomQuat()
qi = randomQuat()
q1 = randomQuat()
q2 = randomQuat()
q3 = randomQuat()
a = q1
b = q2
c = q3

def qfunc(dq,qi,qim1):
    dqim1 = dq[3:6]
    dqi = dq[0:3]
    qi = qdot(qexp(dqi),qi)
    qim1 = qdot(qexp(dqim1),qim1)
    return qlog( qdot(qinv(qim1),qi) )

def qfuncJac(qi,qim1):
    Cim1 = asrl.quat2r(qim1)
    L = invS( qlog(qdot(qinv(qim1),qi) ))
    Jac = np.hstack([Cim1.T,-Cim1.T])
    return dotn(L,Jac)

Jfunvarphi = nd.Jacobian( lambda dq: qfunc(dq,qi,qim1) )



Jfun2 = nd.Jacobian( lambda d: qlog( qdot(qexp(d),q1) ) )

Jfun2(np.array([0,0,0.0]))
invS(qlog(q1))


def S(phi):
    p = math.sqrt(dot(phi.T,phi))
    a = phi/p
    sp2 = math.sin(0.5 * p)
    ax = asrl.crossMx(a)
    px = asrl.crossMx(phi)
    c1 = (-2 * sp2 * sp2) / (p * p)
    c2 = (p - math.sin(p))/(p * p * p)
    S = eye(3) + c1 * px + c2 * np.dot(px,px)
    return S

def S2(phi):
    p = math.sqrt(dot(phi.T,phi))
    a = phi/p
    sp2 = math.sin(0.5 * p)
    sp = math.sin(p)
    ax = asrl.crossMx(a)
    px = asrl.crossMx(phi)
    return eye(3) - (2/p) * sp2 * sp2 * ax + (1/p)*(p - sp)*np.dot(ax,ax)

Jfun4 = nd.Jacobian( lambda d: qexp( d ) )


def dqinv(dq,q1):
    dq1 = qdot(qexp(dq),q1)
    return qinv(dq1)

Jfun5 = nd.Jacobian( lambda d: dqinv(d, q1) )

# These equations show what happens when you substitute
# quat --> Euler Rodriguez for qlog
# and
# Euler Rodriquez --> quat for qexp.
#
# The equations are *MUCH NICER*
#
# Wow, are they ever nice.
def qlog2(q):
    return qeps(q)/qeta(q)
    

def qexp2(aa):
    phi = 2*math.atan(np.linalg.norm(aa))
    eta = math.cos(0.5 * phi)
    q = np.hstack([aa*eta,eta])
    return q

def invS2(p):
    return  ( dot(p,p.T) + eye(3) + asrl.crossMx(p) )

Jfun3 = nd.Jacobian( lambda d: qlog2( qdot(qexp2(d),q1) ) )


    
