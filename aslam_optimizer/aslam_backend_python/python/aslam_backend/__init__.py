# Import the numpy to Eigen type conversion.
import numpy_eigen
import sm
# Import the the C++ exports from your package library.
from libaslam_backend_python import *
# Import other files in the directory
# from mypyfile import *


class TransformationDv(object):
    def __init__(self, transformation, rotationActive=True, translationActive=True ):
        if not type(transformation) == sm.Transformation:
            raise RuntimeError("The transformation must be an sm transformation")
        self.initail_T = transformation
        self.q = RotationQuaternionDv(transformation.q())
        self.q.setActive(rotationActive)
        self.t = EuclideanPointDv(transformation.t())
        self.t.setActive(translationActive)
        self.basic_dv = TransformationBasicDv(self.q.toExpression(), self.t.toExpression())
        self.expression = self.basic_dv.toExpression()
    def toExpression(self):
        return self.expression
    def numDesignVariables(self):
        return 2
    def designVariable(self, i):
        return self.getDesignVariable(i)
    def getDesignVariable(self, i):
        if i == 0:
            return self.q
        elif i == 1:
            return self.t
        else:
            raise RuntimeError("Index out of bounds: %d >= 2" % i)
    def T(self):
        return self.expression.toTransformationMatrix()
