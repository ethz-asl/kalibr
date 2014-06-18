from quaternions import *
import math

class EuclideanSpace :
    def log(self, pos1, pos2) :
        return pos2 - pos1 

    def exp(self, pos, vec) :
        return pos + vec
    
    def getIdentity(self):
        return numpy.array((0, 0, 0))
    
    def product(self, a, b):
        return a + b

    def inv(self, a):
        return -a

class UnitQuaternionManifold:
    def log(self, a, b):
        return qlog(qdot(qinv(a), b))
    
    def exp(self, a, vec):
        return qdot(a, qexp(vec))
    
    def getIdentity(self):
        return numpy.array((0, 0, 0, 1))
    
    def product(self, a, b):
        return qdot(a, b)
    
    def inv(self, a):
        return qinv(a)

    
class SO3DiffManifold:
    def log(self, a, b):
        r = 2 * qlog(qdot(qinv(a), b))
        l = numpy.linalg.norm(r, 1);
        if (l > math.pi) :
            r = r * ((l - 2 * math.pi) / l)  
        return r
    
    def exp(self, a, vec):
        return qdot(a, qexp(2 * vec))
    
    def getIdentity(self):
        return numpy.array((0, 0, 0, 1))

    def product(self, a, b):
        return qdot(a, b)
