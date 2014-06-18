import numpy

def plotPoseSpline(ax, poseSpline, dt=0.1, invert=False, linespec='b-'):
    # generate a 3d curve.
    curve = []
    if invert:
        curve = numpy.array([ poseSpline.inverseTransformation(t)[0:3,3] for t in numpy.append(numpy.arange(poseSpline.t_min(),poseSpline.t_max(),dt),poseSpline.t_max())])
    else:
        curve = numpy.array([ poseSpline.position(t) for t in numpy.append(numpy.arange(poseSpline.t_min(),poseSpline.t_max(),dt),poseSpline.t_max())])
    ax.plot3D(curve[:,0], curve[:,1], curve[:,2],linespec)
    #print "hello"
    #print curve
