from __future__ import print_function #handle print in 2.x python
from sm import PlotCollection
from . import IccPlots as plots
import sm
import numpy as np
import pylab as pl
import sys
import subprocess
import yaml
import time
from matplotlib.backends.backend_pdf import PdfPages
import mpl_toolkits.mplot3d.axes3d as p3
import io
try:
    # Python 2
    from cStringIO import StringIO
except ImportError:
    # Python 3
    from io import StringIO
import matplotlib.patches as patches

# make numpy print prettier
np.set_printoptions(suppress=True)


def plotTrajectory(cself, fno=1, clearFigure=True, title=""):
    f = pl.figure(fno)
    if clearFigure:
        f.clf()
    f.suptitle(title)

    size = 0.05
    a3d = f.add_subplot(111, projection='3d')

    # get times we will evaulate at (fixed frequency)
    imu = cself.ImuList[0]
    bodyspline = cself.poseDv.spline()
    times_imu = np.array([im.stamp.toSec() + imu.timeOffset for im in imu.imuData \
                      if im.stamp.toSec() + imu.timeOffset > bodyspline.t_min() \
                      and im.stamp.toSec() + imu.timeOffset < bodyspline.t_max() ])
    times = np.arange(np.min(times_imu), np.max(times_imu), 1.0/10.0)
    
    #plot each pose
    traj_max = np.array([-9999.0, -9999.0, -9999.0])
    traj_min = np.array([9999.0, 9999.0, 9999.0])
    T_last = None
    for time in times:
        position =  bodyspline.position(time)
        orientation = sm.r2quat(bodyspline.orientation(time))
        T = sm.Transformation(orientation, position)
        sm.plotCoordinateFrame(a3d, T.T(), size=size)
        # record min max
        traj_max = np.maximum(traj_max, position)
        traj_min = np.minimum(traj_min, position)
        # compute relative change between
        if T_last != None:
            pos1 = T_last.t()
            pos2 = T.t()
            a3d.plot3D([pos1[0], pos2[0]],[pos1[1], pos2[1]],[pos1[2], pos2[2]],'k-', linewidth=1)
        T_last = T

    #TODO: should also plot the target board here (might need to transform into imu0 grav?)

    a3d.auto_scale_xyz([traj_min[0]-size, traj_max[0]+size], [traj_min[1]-size, traj_max[1]+size], [traj_min[2]-size, traj_max[2]+size])


def printErrorStatistics(cself, dest=sys.stdout):
    # Reprojection errors
    print("Normalized Residuals\n----------------------------", file=dest)
    for cidx, cam in enumerate(cself.CameraChain.camList):
        if len(cam.allReprojectionErrors)>0:
            e2 = np.array([ np.sqrt(rerr.evaluateError()) for reprojectionErrors in cam.allReprojectionErrors for rerr in reprojectionErrors])
            print("Reprojection error (cam{0}):     mean {1}, median {2}, std: {3}".format(cidx, np.mean(e2), np.median(e2), np.std(e2) ), file=dest)
        else:
            print("Reprojection error (cam{0}):     no corners".format(cidx), file=dest)
    
    for iidx, imu in enumerate(cself.ImuList):
        # Gyro errors
        e2 = np.array([ np.sqrt(e.evaluateError()) for e in imu.gyroErrors ])
        print("Gyroscope error (imu{0}):        mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)
        # Accelerometer errors
        e2 = np.array([ np.sqrt(e.evaluateError()) for e in imu.accelErrors ])
        print("Accelerometer error (imu{0}):    mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)

    print("", file=dest)
    print("Residuals\n----------------------------", file=dest)
    for cidx, cam in enumerate(cself.CameraChain.camList):
        if len(cam.allReprojectionErrors)>0:
            e2 = np.array([ np.linalg.norm(rerr.error()) for reprojectionErrors in cam.allReprojectionErrors for rerr in reprojectionErrors])
            print("Reprojection error (cam{0}) [px]:     mean {1}, median {2}, std: {3}".format(cidx, np.mean(e2), np.median(e2), np.std(e2) ), file=dest)
        else:
            print("Reprojection error (cam{0}) [px]:     no corners".format(cidx), file=dest)
    
    for iidx, imu in enumerate(cself.ImuList):
        # Gyro errors
        e2 = np.array([ np.linalg.norm(e.error()) for e in imu.gyroErrors ])
        print("Gyroscope error (imu{0}) [rad/s]:     mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)
        # Accelerometer errors
        e2 = np.array([ np.linalg.norm(e.error()) for e in imu.accelErrors ])
        print("Accelerometer error (imu{0}) [m/s^2]: mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)

def printGravity(cself):
    print("")
    print("Gravity vector: (in target coordinates): [m/s^2]")
    print(cself.gravityDv.toEuclidean())

def printResults(cself, withCov=False):
    nCams = len(cself.CameraChain.camList)
    for camNr in range(0,nCams):
        T_cam_b = cself.CameraChain.getResultTrafoImuToCam(camNr)

        print("")
        print("Transformation T_cam{0}_imu0 (imu0 to cam{0}, T_ci): ".format(camNr))
        if withCov and camNr==0:
            print("    quaternion: ", T_cam_b.q(), " +- ", cself.std_trafo_ic[0:3])
            print("    translation: ", T_cam_b.t(), " +- ", cself.std_trafo_ic[3:])
        print(T_cam_b.T())
        
        if not cself.noTimeCalibration:
            print("")
            print("cam{0} to imu0 time: [s] (t_imu = t_cam + shift)".format(camNr))
            print(cself.CameraChain.getResultTimeShift(camNr), end=' ')
            
            if withCov:
                print(" +- ", cself.std_times[camNr])
            else:
                print("")

    print("")
    for (imuNr, imu) in enumerate(cself.ImuList):
        print("IMU{0}:\n".format(imuNr), "----------------------------")
        imu.getImuConfig().printDetails()
            
def printBaselines(self):
    #print all baselines in the camera chain
    if nCams > 1:
        for camNr in range(0,nCams-1):
            T, baseline = cself.CameraChain.getResultBaseline(camNr, camNr+1)
            
            if cself.CameraChain.camList[camNr+1].T_extrinsic_fixed:
                isFixed = "(fixed to external data)"
            else:
                isFixed = ""
            
            print("")
            print("Baseline (cam{0} to cam{1}): [m] {2}".format(camNr, camNr+1, isFixed))
            print(T.T())
            print(baseline, "[m]")
    


def generateReport(cself, filename="report.pdf", showOnScreen=True):
    figs = list()
    plotter = PlotCollection.PlotCollection("Calibration report")
    offset = 3010
    
    #Output calibration results in text form.
    sstream = StringIO()
    printResultTxt(cself, sstream)
    text = [line for line in StringIO(sstream.getvalue())]
    linesPerPage = 35
    
    while True:
        fig = pl.figure(offset)
        offset += 1

        left, width = .05, 1.
        bottom, height = -.05, 1.
        right = left + width
        top = bottom + height
        
        ax = fig.add_axes([.0, .0, 1., 1.])
        # axes coordinates are 0,0 is bottom left and 1,1 is upper right
        p = patches.Rectangle((left, bottom), width, height, fill=False, transform=ax.transAxes, \
                                 clip_on=False, edgecolor="none")
        ax.add_patch(p)
        pl.axis('off')

        printText = lambda t: ax.text(left, top, t, fontsize=7, \
                                     horizontalalignment='left', verticalalignment='top',\
                                     transform=ax.transAxes, wrap=True)
        
        if len(text) > linesPerPage:
            printText("".join(text[0:linesPerPage]))
            figs.append(fig)
            text = text[linesPerPage:]
        else:
            printText("".join(text[0:]))
            figs.append(fig)
            break
    
    #plot trajectory
    f=pl.figure(1003)
    title="imu0: estimated poses"
    plotTrajectory(cself, fno=f.number, clearFigure=False, title=title)
    plotter.add_figure(title, f)
    figs.append(f)
    
    #plot imu stuff (if we have imus)
    for iidx, imu in enumerate(cself.ImuList):

        f = pl.figure(offset+iidx)
        plots.plotIMURates(cself, iidx, fno=f.number, noShow=True)
        plotter.add_figure("imu{0}: measurement rates".format(iidx), f)
        figs.append(f)
        offset += len(cself.ImuList)

        f = pl.figure(offset+iidx)
        plots.plotAccelerations(cself, iidx, fno=f.number, noShow=True)
        plotter.add_figure("imu{0}: accelerations".format(iidx), f)
        figs.append(f)
        offset += len(cself.ImuList)

        f = pl.figure(offset+iidx)
        plots.plotAccelErrorPerAxis(cself, iidx, fno=f.number, noShow=True)
        plotter.add_figure("imu{0}: acceleration error".format(iidx), f)
        figs.append(f)
        offset += len(cself.ImuList)

        f = pl.figure(offset+iidx)
        plots.plotAccelBias(cself, iidx, fno=f.number, noShow=True)
        plotter.add_figure("imu{0}: accelerometer bias".format(iidx), f)
        figs.append(f)
        offset += len(cself.ImuList)

        f = pl.figure(offset+iidx)
        plots.plotAngularVelocities(cself, iidx, fno=f.number, noShow=True)
        plotter.add_figure("imu{0}: angular velocities".format(iidx), f)
        figs.append(f)
        offset += len(cself.ImuList)

        f = pl.figure(offset+iidx)
        plots.plotGyroErrorPerAxis(cself, iidx, fno=f.number, noShow=True)
        plotter.add_figure("imu{0}: angular velocity error".format(iidx), f)
        figs.append(f)
        offset += len(cself.ImuList)

        f = pl.figure(offset+iidx)
        plots.plotAngularVelocityBias(cself, iidx, fno=f.number, noShow=True)
        plotter.add_figure("imu{0}: gyroscope bias".format(iidx), f)
        figs.append(f)
        offset += len(cself.ImuList)

    #plot cam stuff
    if cself.CameraChain:        
        for cidx, cam in enumerate(cself.CameraChain.camList):
            f = pl.figure(offset+cidx)
            title="cam{0}: reprojection errors".format(cidx);
            plots.plotReprojectionScatter(cself, cidx, fno=f.number, noShow=True, title=title)
            plotter.add_figure(title, f)
            figs.append(f)
            offset += len(cself.CameraChain.camList)

    #write to pdf
    pdf=PdfPages(filename)
    for fig in figs:
        pdf.savefig(fig)
    pdf.close()

    if showOnScreen:
        plotter.show()

def exportPoses(cself, filename="poses_imu0.csv"):
    
    # Append our header, and select times at IMU rate
    f = open(filename, 'w')
    print("#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []", file=f)
    imu = cself.ImuList[0]
    bodyspline = cself.poseDv.spline()
    times = np.array([im.stamp.toSec() + imu.timeOffset for im in imu.imuData \
                      if im.stamp.toSec() + imu.timeOffset > bodyspline.t_min() \
                      and im.stamp.toSec() + imu.timeOffset < bodyspline.t_max() ])

    # Times are in nanoseconds -> convert to seconds
    # Use the ETH groundtruth csv format [t,q,p,v,bg,ba]
    for time in times:
        position =  bodyspline.position(time)
        orientation = sm.r2quat(bodyspline.orientation(time))
        print("{:.0f},".format(1e9 * time) + ",".join(map("{:.6f}".format, position)) \
               + "," + ",".join(map("{:.6f}".format, orientation)) , file=f)

def saveResultTxt(cself, filename='cam_imu_result.txt'):
    f = open(filename, 'w')
    printResultTxt(cself, stream=f)

def printResultTxt(cself, stream=sys.stdout):
    
    print("Calibration results", file=stream)
    print("===================", file=stream)   
    printErrorStatistics(cself, stream)
  
    # Calibration results
    nCams = len(cself.CameraChain.camList)
    for camNr in range(0,nCams):
        T = cself.CameraChain.getResultTrafoImuToCam(camNr)
        print("", file=stream)
        print("Transformation (cam{0}):".format(camNr), file=stream)
        print("-----------------------", file=stream)
        print("T_ci:  (imu0 to cam{0}): ".format(camNr), file=stream)   
        print(T.T(), file=stream)
        print("", file=stream)
        print("T_ic:  (cam{0} to imu0): ".format(camNr), file=stream)   
        print(T.inverse().T(), file=stream)
    
        # Time
        print("", file=stream)
        print("timeshift cam{0} to imu0: [s] (t_imu = t_cam + shift)".format(camNr), file=stream)
        print(cself.CameraChain.getResultTimeShift(camNr), file=stream)
        print("", file=stream)

    #print all baselines in the camera chain
    if nCams > 1:
        print("Baselines:", file=stream)
        print("----------", file=stream)

        for camNr in range(0,nCams-1):
            T, baseline = cself.CameraChain.getResultBaseline(camNr, camNr+1)
            print("Baseline (cam{0} to cam{1}): ".format(camNr, camNr+1), file=stream)
            print(T.T(), file=stream)
            print("baseline norm: ", baseline,  "[m]", file=stream)
            print("", file=stream)
    
    # Gravity
    print("", file=stream)
    print("Gravity vector in target coords: [m/s^2]", file=stream)
    print(cself.gravityDv.toEuclidean(), file=stream)
    
    print("", file=stream)
    print("", file=stream)
    print("Calibration configuration", file=stream)
    print("=========================", file=stream)
    print("", file=stream)

    for camNr, cam in enumerate( cself.CameraChain.camList ):
        print("cam{0}".format(camNr), file=stream)
        print("-----", file=stream)
        cam.camConfig.printDetails(stream)
        cam.targetConfig.printDetails(stream)
        print("", file=stream)
    
    print("", file=stream)
    print("", file=stream)
    print("IMU configuration", file=stream)
    print("=================", file=stream)
    print("", file=stream)
    for (imuNr, imu) in enumerate(cself.ImuList):
        print("IMU{0}:\n".format(imuNr), "----------------------------", file=stream)
        imu.getImuConfig().printDetails(stream)
        print("", file=stream)
