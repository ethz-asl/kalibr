import visual
import thread
import time
import numpy

class Manifold :
    class SceneObject(object) :
        def __init__(self, visual):
            self._visual = visual
            
        def setOpacity(self, opacity):
            self._visual.opacity = opacity
        def setRadius(self, radius):
            self._visual.radius = radius
        
    class point(SceneObject) :
        def __init__(self, pos, color, manifold):
            self.__pos = pos
            super(self.__class__, self).__init__(visual.sphere(radius = 0.1, pos = manifold.getCurrentPosVec(pos), color = color, display = manifold.display))
            self._visual.point = self
            self.__manifold = manifold
            
        def __str__(self):
            return str(self.__pos)
        
        def setPos(self, pos):
            self.__pos = pos
            self.updatePos()
            
        def updatePos(self):
            self._visual.pos = self.__manifold.getCurrentPosVec(self.__pos)

            
    class curve(SceneObject) :
        def __init__(self, points, color, manifold):
            self.__points = points
            super(self.__class__, self).__init__(visual.curve(radius = 0.01, pos = [manifold.getCurrentPosVec(pos) for pos in points], color = color, display = manifold.display))
            self.__manifold = manifold
            
        def setPos(self, points):
            self.__points = points
            self.updatePos()
            
        def updatePos(self):
            self._visual.pos = [self.__manifold.getCurrentPosVec(pos) for pos in self.__points]
    
    def __init__(self, startpos, geometry, title="threemanifoldView"):
        self.display = visual.display(title = title)
        self.objects = []
        self.__currentpos = startpos
        self.__geometry = geometry
        self.__stepsize = 0.25

    def addPoint(self, pos, color = visual.color.white):
        p = Manifold.point(pos, color, self)
        self.objects.append(p)
        return p
    
    def addCurve(self, points, color = visual.color.white):
        p = Manifold.curve(points, color, self)
        self.objects.append(p)
        return p
    
    
    def setZeroPointEnabled(self, enabled):
        try:
            self.__zeroPointSphere.hidden = enabled
        except:
            if enabled : 
                self.__zeroPointSphere = visual.sphere(radius = 0.1, opacity = 0.5) 
     
    def getCurrentPosVec(self, pos):
        return self.__geometry.log(self.__currentpos, pos)
    
    def getCurrentVecPos(self, vec):
        return self.__geometry.exp(self.__currentpos, vec)

    def setCurrentPos(self, pos):
        self.__currentpos = pos
        for p in self.objects :
            p.updatePos()
#        print self.__currentpos
    
    def __interactionThread(self):
        while True :    
            if self.display.kb.keys: # event waiting to be processed?
                s = self.display.kb.getkey() # get keyboard info
                handler = self.__keyHandler.get(s)
                if handler:
                    handler()
                else:
                    print "'" + s + "'"
                    print self.__keyHandler
            elif self.display.mouse.events:
                ev = self.display.mouse.getevent()
                if ev.press :
                    try :
                        p = ev.pick.point
                        print p
                    except :
                        pass
            else:
                time.sleep(0.01)
            
    def __step(self, vec):
        self.setCurrentPos(self.getCurrentVecPos(numpy.array(vec) * self.__stepsize))
        
    def getKeyHandler(self):
        return self.__keyHandler;
    
    def setKeyHandler(self, handler):
        self.__keyHandler = handler;
    
    def startInteractionThread(self):
        try :
            return self.__thread
        except :
            self.__keyHandler = { 
                    "left" : lambda : self.__step((-1, 0, 0)),
                    "right" : lambda : self.__step((1, 0, 0)),
                    "page up" : lambda : self.__step((0, 1, 0)),
                    "page down" : lambda : self.__step((0, -1, 0)),
                    "up" : lambda : self.__step((0, 0, 1)),
                    "down" : lambda : self.__step((0, 0, -1)),
                }
                
            self.__thread = thread.start_new_thread(self.__interactionThread, ())
            return self.__thread
