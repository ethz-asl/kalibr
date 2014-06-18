import numpy
import pylab as p

import matplotlib.axes as axes

def plotCoordinateFrame(axis, T_0f, size=1, linewidth=3):
    """Plot a coordinate frame on a 3d axis. In the resulting plot,
    x = red, y = green, z = blue.
    
    plotCoordinateFrame(axis, T_0f, size=1, linewidth=3)

    Arguments:
    axis: an axis of type matplotlib.axes.Axes3D
    T_0f: The 4x4 transformation matrix that takes points from the frame of interest, to the plotting frame
    size: the length of each line in the coordinate frame
    linewidth: the width of each line in the coordinate frame

    Usage is a bit irritating:
    import mpl_toolkits.mplot3d.axes3d as p3
    import pylab as pl

    f1 = pl.figure(1)
    # old syntax
    # a3d = p3.Axes3D(f1)
    # new syntax
    a3d = f1.add_subplot(111, projection='3d')
    # ... Fill in T_0f, the 4x4 transformation matrix
    plotCoordinateFrame(a3d, T_0f)

    see http://matplotlib.sourceforge.net/mpl_toolkits/mplot3d/tutorial.html for more details
    """
    # \todo fix this check.
    #if type(axis) != axes.Axes3D:
    #    raise TypeError("axis argument is the wrong type. Expected matplotlib.axes.Axes3D, got %s" % (type(axis)))

    p_f = numpy.array([ [ 0,0,0,1], [size,0,0,1], [0,size,0,1], [0,0,size,1]]).T;
    p_0 = numpy.dot(T_0f,p_f)
    # X-axis

    X = numpy.append( [p_0[:,0].T] , [p_0[:,1].T], axis=0 )
    Y = numpy.append( [p_0[:,0].T] , [p_0[:,2].T], axis=0 )
    Z = numpy.append( [p_0[:,0].T] , [p_0[:,3].T], axis=0 )
    axis.plot3D(X[:,0],X[:,1],X[:,2],'r-', linewidth=linewidth)
    axis.plot3D(Y[:,0],Y[:,1],Y[:,2],'g-', linewidth=linewidth)
    axis.plot3D(Z[:,0],Z[:,1],Z[:,2],'b-', linewidth=linewidth)
    
