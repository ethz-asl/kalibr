import sm
import numpy


def qdot(q1,q2):
    return numpy.dot(sm.quatPlus(q1),q2)

def qinv(q):
    return sm.quatInv(q)

def qlog(q):
    return sm.quat2AxisAngle(q)

def qexp(a):
    return sm.axisAngle2quat(a)