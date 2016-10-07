import bsplines
import numpy

def createExpBSpline(geometry, splineOrder, time_min, time_max, segmentNumber):
        bs = bsplines.BSpline(splineOrder)
        bs.initConstantSpline(time_min, time_max, segmentNumber, numpy.array((0,)))
        return ExponentialBSpline(geometry, bs)

class ExponentialBSpline:
    def __init__(self, geometry, bspline):
        self.__geometry = geometry
        self.__bs = bspline
        self.setControlVertices(bspline.coefficients().transpose())
        
    def setControlVertices(self, controlVertices):
        self.__controlVertices = controlVertices
    
    def eval(self, t):
        bs = self.__bs
        cumulativeBi = bs.getLocalCumulativeBi(t);
        ci = bs.localVvCoefficientVectorIndices(t)
        qci = self.__controlVertices[ci]
        rval = qci[0]
        for i in range(0,len(ci) - 1):
            ldqi = self.__geometry.log(qci[i],qci[i+1])
            rval= self.__geometry.exp(rval, ldqi * cumulativeBi[i+1])
        return rval

    def numVvCoefficients(self):
        return self.__bs.numVvCoefficients()
    
    def getBiFunction(self,t):
        return self.__bs.getBiFunction(t)

    def getBSpline(self):
        return self.__bs
    