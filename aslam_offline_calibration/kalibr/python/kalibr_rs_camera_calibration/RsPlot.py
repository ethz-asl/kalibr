import sm
import numpy as np
import pylab as pl

def plotSpline(spline, splineB = None):
    """Plots a spline over the full range of times.

    The orientations are shown as EulerAnglesYawPitchRoll.

    Args:
        spline (BsplinePose): a pose describing spline
    """
    ap = sm.EulerAnglesYawPitchRoll()
    t1 = spline.t_min()
    t2 = spline.t_max()
    times = np.linspace(t1, t2, 1000)

    pos = []
    ori = []
    for t in times:
        T = spline.transformation(t)
        pos.append(T[:,3])
        ori.append(ap.rotationMatrixToParameters(T[:3,:3]))

    if (splineB is not None):
        posB = []
        oriB = []
        for t in times:
            T = splineB.transformation(t)
            posB.append(T[:,3])
            oriB.append(ap.rotationMatrixToParameters(T[:3,:3]))
        posB = np.array(posB)
        oriB = np.array(oriB)

    pos = np.array(pos)
    ori = np.array(ori)

    pl.figure()
    pl.subplot(311)
    pl.title('Position')
    pl.plot(times, (pos[:,0]))
    if (splineB is not None):
        pl.plot(times, (posB[:,0]))

    pl.subplot(312)
    pl.plot(times, (pos[:,1]))
    if (splineB is not None):
        pl.plot(times, (posB[:,1]))

    pl.subplot(313)
    pl.plot(times, (pos[:,2]))
    if (splineB is not None):
        pl.plot(times, (posB[:,2]))

    pl.figure()
    pl.subplot(311)
    pl.title('Orientation')
    pl.plot(times, (ori[:,0]))
    if (splineB is not None):
        pl.plot(times, (oriB[:,0]))

    pl.subplot(312)
    pl.plot(times, (ori[:,1]))
    if (splineB is not None):
        pl.plot(times, (oriB[:,1]))

    pl.subplot(313)
    pl.plot(times, (ori[:,2]))
    if (splineB is not None):
        pl.plot(times, (oriB[:,2]))

    pl.show()



def plotSplineValues(spline, splineB = None):
    """Plots a spline over the full range of times.

    Args:
        spline (BsplinePose): a pose describing spline
    """
    t1 = spline.t_min()
    t2 = spline.t_max()
    times = np.linspace(t1, t2, 1000)

    pos = []
    for t in times:
        v = spline.eval(t)
        pos.append(v)

    if (splineB is not None):
        posB = []
        for t in times:
            v = splineB.eval(t)
            posB.append(v)
        posB = np.array(posB)

    pos = np.array(pos)

    pl.figure()
    pl.subplot(311)
    pl.title('Position')
    pl.plot(times, (pos[:,0]))
    if (splineB is not None):
        pl.plot(times, (posB[:,0]))

    pl.subplot(312)
    pl.plot(times, (pos[:,1]))
    if (splineB is not None):
        pl.plot(times, (posB[:,1]))

    pl.subplot(313)
    pl.plot(times, (pos[:,2]))
    if (splineB is not None):
        pl.plot(times, (posB[:,2]))

    pl.figure()
    pl.subplot(311)
    pl.title('Orientation')
    pl.plot(times, (pos[:,3]))
    if (splineB is not None):
        pl.plot(times, (posB[:,3]))

    pl.subplot(312)
    pl.plot(times, (pos[:,4]))
    if (splineB is not None):
        pl.plot(times, (posB[:,4]))

    pl.subplot(313)
    pl.plot(times, (pos[:,5]))
    if (splineB is not None):
        pl.plot(times, (posB[:,5]))

    pl.show()

