import roslib
roslib.load_manifest('splines')
import asrl_splines as asp
import numpy as np
import pylab as pl
import math
import asrl

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




def qlog(q):
    return asrl.quat2AxisAngle(q)

def qexp(aa):
    return asrl.axisAngle2quat(aa)

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

def qlog2(q):
    return qeps(q)/qeta(q)
    

def qexp2(aa):
    phi = 2*math.atan(np.linalg.norm(aa))
    eta = math.cos(0.5 * phi)
    q = np.hstack([aa*eta,eta])
    return q


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
    #print bi
    #print sum(bi)
    #print ci
    for i in range(0,len(ci) - 1):
        dqi = qdot(qinv(qci[:,i]),qci[:,i+1])
        ldqi = qlog2(dqi)
        tildeB = np.sum(bi[i+1:])
        rval= qdot(rval, qexp2(ldqi * tildeB))
        

    return rval



if True:
    T = np.arange(bs.t_min()+1e-6,bs.t_max()-1e-6,(bs.t_max() - bs.t_min())/200)    
    #qval = np.array( [ qdot(qleft,qdot(cumQuat(bs,t,qc),qright))[:,0] for t in T ] )
    #qvalt = np.array( [ cumQuat(bs,t,qct) for t in T ] )
    qval = np.array( [ cumQuat(bs,t,qct) for t in T ] )
    qvalt = np.array( [ cumQuat2(bs,t,qct) for t in T ] )

    
    nm = np.sum(qval * qval,1)
    pl.figure(3)
    pl.clf()
    pl.plot(T,qval,linewidth=4)
    pl.plot(T,qvalt,linewidth=3,ls='--')
    #pl.plot(T,nm,linewidth=4)
    #pl.plot(k[3:-3],qc.T,'.k',markersize=4)

if True:
    C = asrl.axisAngle2r(np.random.random([3,1])*10)
    R = asrl.axisAngle2r(np.random.random([3,1])*10)
    b = 10.3

    C1 = cexp(b * clog( np.dot(R.T, np.dot( C, R))))
    C2 = np.dot( R.T, np.dot( cexp(b * clog(C) ), R))

    print C1 - C2
