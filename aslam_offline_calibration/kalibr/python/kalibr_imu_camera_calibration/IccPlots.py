import numpy as np
import pylab as pl

def plotGyroError(cself, iidx, fno=1, clearFigure=True, noShow=False):
    errors = np.array([re.getRawSquaredError() for re in  cself.ImuList[iidx].gyroErrors])
   
    f = pl.figure(fno)
    if clearFigure:
        f.clf()
    f.suptitle("imu{0}: angular velocities error".format(iidx))
    
    pl.subplot(2, 1, 1)
    pl.plot(errors)
    pl.xlabel('error index')
    pl.ylabel('error (rad/sec) squared')
    pl.grid('on')
    
    #only plot till 5*sigma  (output formatting...)
    sigma=np.std(errors)
    errors = errors[ errors < 5*sigma ]
    
    pl.subplot(2, 1, 2)
    pl.hist(errors, len(errors)/100)
    pl.xlabel('error (rad/sec) squared')
    pl.ylabel('error index')
    pl.grid('on')

def plotAccelError(cself, iidx, fno=1, clearFigure=True, noShow=False):
    errors = np.array([re.getRawSquaredError() for re in cself.ImuList[iidx].accelErrors])
   
    f = pl.figure(fno)
    if clearFigure:
        f.clf()
    f.suptitle("imu{0}: acceleration error".format(iidx))
        
    pl.subplot(2, 1, 1)
    pl.plot(errors)
    pl.xlabel('error index')
    pl.ylabel('(m/sec*sec) squared')
    pl.grid('on')
    
    #only plot till 5*sigma  (output formatting...)
    sigma=np.std(errors)
    errors = errors[ errors < 5*sigma ]
    
    pl.subplot(2, 1, 2)
    pl.hist(errors, len(errors)/100)
    pl.xlabel('(m/sec*sec) squared')
    pl.ylabel('Error Number')
    pl.grid('on')

def plotAccelBias(cself, imu_idx, fno=1, clearFigure=True, noShow=False):
    imu = cself.ImuList[imu_idx]
    bias = imu.accelBiasDv.spline()
    times = np.array([im.stamp.toSec() for im in imu.imuData if im.stamp.toSec() > bias.t_min() and im.stamp.toSec() < bias.t_max() ])
    acc_bias_spline = np.array([bias.evalD(t,0) for t in times]).T
    times = times - times[0]     #remove time offset

    plotVectorOverTime(times, acc_bias_spline, 
                       title="imu{0}: estimated accelerometer bias (imu frame)".format(imu_idx), 
                       ylabel="bias ($m/s^2$)", 
                       fno=fno, clearFigure=clearFigure, noShow=noShow)
    
def plotAngularVelocityBias(cself, imu_idx, fno=1, clearFigure=True, noShow=False):
    imu = cself.ImuList[imu_idx]
    bias = imu.gyroBiasDv.spline()
    times = np.array([im.stamp.toSec() for im in imu.imuData if im.stamp.toSec() > bias.t_min() and im.stamp.toSec() < bias.t_max() ])
    avimubias_spline = np.array([bias.evalD(t,0) for t in times]).T
    times = times - times[0]     #remove time offset
    
    plotVectorOverTime(times, avimubias_spline, 
                       title="imu{0}: estimated gyro bias (imu frame)".format(imu_idx), 
                       ylabel="bias ($rad/s$)", 
                       fno=fno, clearFigure=clearFigure, noShow=noShow)

#plots angular velocity of the body fixed spline versus all imu measurements
def plotAngularVelocities(cself, iidx, fno=1, clearFigure=True, noShow=False):
    #predicted (over the time of the imu)
    imu = cself.ImuList[iidx]
    bodyspline = cself.poseDv.spline()   
    times = np.array([im.stamp.toSec() for im in imu.imuData if im.stamp.toSec() > bodyspline.t_min() and im.stamp.toSec() < bodyspline.t_max() ])
    predictedAng_body =  np.array([err.getPredictedMeasurement() for err in imu.gyroErrors]).T
    
    #transform the measurements to the body frame
    #not neccessray for imu0 as it is aligned with the spline
    measuredAng_body =  np.array([err.getMeasurement() for err in imu.gyroErrors]).T

    #remove time offset
    times = times - times[0]
    
    #plot the predicted measurements
    plotVectorOverTime(times, predictedAng_body, 
                       title="Comparison of predicted and measured angular velocities (body frame)", 
                       ylabel="ang. velocity ($rad/s$)", 
                       label="est. bodyspline",
                       fno=fno, clearFigure=clearFigure, noShow=noShow, lw=3)
    
    #plot measurements
    for r in range(0,3):
        ax=pl.subplot(3, 1, r+1)
        pl.plot(times, measuredAng_body[r,:], 'x', lw=1, label="imu{0}".format(iidx))
        pl.legend()

def plotAccelerations(cself, iidx, fno=1, clearFigure=True, noShow=False):   
    #predicted 
    imu = cself.ImuList[iidx]
    bodyspline = cself.poseDv.spline()   
    times = np.array([im.stamp.toSec() for im in imu.imuData if im.stamp.toSec() > bodyspline.t_min() and im.stamp.toSec() < bodyspline.t_max() ])
    predicetedAccel_body =  np.array([err.getPredictedMeasurement() for err in imu.accelErrors]).T
    
    #transform accelerations from imu to body frame (on fixed body and geometry was estimated...)
    #works for imu0 as it is aligned with the spline
    #TODO(schneith): implement the fixed-body acceleration transformation 
    measuredAccel_imu =  np.array([err.getMeasurement() for err in imu.accelErrors]).T
    measuredAccel_body = measuredAccel_imu
    
    #remove time offset
    times = times - times[0] 
    
    #plot the predicted measurements
    plotVectorOverTime(times, predicetedAccel_body, 
                       title="Comparison of predicted and measured specific force (imu0 frame)", 
                       ylabel="specific force ($m/s^2$)", 
                       label="est. bodyspline",
                       fno=fno, clearFigure=clearFigure, noShow=noShow, lw=3)
    
    #plot the measurements
    for r in range(0,3):
        ax=pl.subplot(3, 1, r+1)
        pl.plot(times, measuredAccel_body[r,:], 'x', lw=1, label="imu{0}".format(iidx))
        pl.legend()

def plotVectorOverTime(times, values, title="", ylabel="", label="", fno=1, clearFigure=True, noShow=False, lw=3):
    f = pl.figure(fno)
    if clearFigure:
        f.clf()
    f.suptitle(title)
    for r in range(0,3):
        pl.subplot(3, 1, r+1)
        pl.plot(times, values[r,:], 'b-', lw=lw, label=label)
        pl.grid('on')
        pl.xlabel("time (s)")
        pl.ylabel(ylabel)
        if label is not "":
            pl.legend()

def plotReprojectionScatter(cself, cam_id, fno=1, clearFigure=True, noShow=False, title=""):
    cam = cself.CameraChain.camList[cam_id]
    
    #create figure
    f = pl.figure(fno)
    if clearFigure:    
        f.clf()
    f.suptitle(title)
    
    numImages = len(cam.allReprojectionErrors)
    values = np.arange(numImages)/np.double(numImages)
    cmap = pl.cm.jet(values,alpha=0.5)

    #reprojection errors scatter plot
    for image_id, rerrs_image in enumerate(cam.allReprojectionErrors):
        color = cmap[image_id,:]
        rerrs = np.array([rerr.error() for rerr in rerrs_image])  
        pl.plot(rerrs[:,0], rerrs[:,1], 'x', lw=3, mew=3, color=color)

    pl.axis('equal')
    pl.grid('on')
    pl.xlabel('error x (pix)')
    pl.ylabel('error y (pix)')
    SM = pl.cm.ScalarMappable(pl.cm.colors.Normalize(0.0,numImages), pl.cm.jet)
    SM.set_array(np.arange(numImages));
    cb = pl.colorbar(SM)
    cb.set_label('image index')
    if not noShow:
        pl.show()

class CameraPlot:
    def __init__(self, fig,  targetPoints, camSize):
        self.initialized = False
        #get the data
        self.targetPoints = targetPoints
        self.camSize = camSize
        self.fig = fig
        #setup the figure
        self.setupFigure()
        self.plot3Dgrid()
        #initialize camerea
        T = np.eye(4,4)
        self.plot3DCamera(T)
        
    def setupFigure(self):
        #interactive mode
        pl.ion()
        #hack to enforce axis equal (matplotlib doesn't support that)
        self.ax.set_aspect('equal')
        MAX = 1
        for direction in (-1, 1):
            for point in np.diag(direction * MAX * np.array([1,1,1])):
                self.ax.plot([point[0]], [point[1]], [point[2]], 'w')
        self.fig.show()
        
    def plot3Dgrid(self):
        #draw target corners        
        for i in range(0, len(self.targetPoints) ):
            self.ax.scatter(self.targetPoints[i,0], self.targetPoints[i,1], self.targetPoints[i,2],color="g",s=1)
        
        self.ax.plot([0,self.targetPoints[-1,0]],[0,0],[0,0], color="r")
        self.ax.plot([0,0],[0,self.targetPoints[-1,1]],[0,0], color="g")
        self.ax.plot([0,0],[0,0],[0,self.targetPoints[-1,0]], color="b")
 
    def plot3DCamera(self, T):
        #transform affine
        ori = T * np.matrix([[0],[0],[0],[1]])
        v1 =  T * np.matrix([[self.camSize],[0],[0],[1]])
        v2 =  T * np.matrix([[0],[self.camSize],[0],[1]])
        v3 =  T * np.matrix([[0],[0],[self.camSize],[1]])
        
        #initialize objects
        if not self.initialized:
            self.cam_x = self.ax.plot(np.squeeze([ori[0], v1[0]]), np.squeeze([ori[1], v1[1]]), np.squeeze([ori[2], v1[2]]), color="r")
            self.cam_y = self.ax.plot(np.squeeze([ori[0], v2[0]]), np.squeeze([ori[1], v2[1]]), np.squeeze([ori[2], v2[2]]), color="g")
            self.cam_z = self.ax.plot(np.squeeze([ori[0], v3[0]]), np.squeeze([ori[1], v3[1]]), np.squeeze([ori[2], v3[2]]), color="b")
            self.initialized = True
            
        else:
            xy=np.squeeze([ori[0:2], v1[0:2]]).transpose()
            z=np.squeeze([ori[2], v1[2]]).transpose()
            self.cam_x[0].set_data(xy)
            self.cam_x[0].set_3d_properties(z)
            xy=np.squeeze([ori[0:2], v2[0:2]]).transpose()
            z=np.squeeze([ori[2], v2[2]]).transpose()
            self.cam_y[0].set_data(xy)
            self.cam_y[0].set_3d_properties(z)
            xy=np.squeeze([ori[0:2], v3[0:2]]).transpose()
            z=np.squeeze([ori[2], v3[2]]).transpose()
            self.cam_z[0].set_data(xy)
            self.cam_z[0].set_3d_properties(z)
            pl.pause(0.00001)