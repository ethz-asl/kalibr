from sm import PlotCollection
import IccPlots as plots
import numpy as np
import pylab as pl
import sys
import subprocess
import yaml
import time
from matplotlib.backends.backend_pdf import PdfPages
import StringIO
import matplotlib.patches as patches

def printErrorStatistics(cself, dest=sys.stdout):
    # Reprojection errors
    print >> dest, "Normalized Residuals\n----------------------------"
    for cidx, cam in enumerate(cself.CameraChain.camList):
        if len(cam.allReprojectionErrors)>0:
            e2 = np.array([ np.sqrt(rerr.evaluateError()) for reprojectionErrors in cam.allReprojectionErrors for rerr in reprojectionErrors])
            print >> dest, "Reprojection error (cam{0}):     mean {1}, median {2}, std: {3}".format(cidx, np.mean(e2), np.median(e2), np.std(e2) )
        else:
            print >> dest, "Reprojection error (cam{0}):     no corners".format(cidx)
    
    for iidx, imu in enumerate(cself.ImuList):
        # Gyro errors
        e2 = np.array([ np.sqrt(e.evaluateError()) for e in imu.gyroErrors ])
        print >> dest, "Gyroscope error (imu{0}):        mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2))
        # Accelerometer errors
        e2 = np.array([ np.sqrt(e.evaluateError()) for e in imu.accelErrors ])
        print >> dest, "Accelerometer error (imu{0}):    mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2))

    print >> dest, ""
    print >> dest, "Residuals\n----------------------------"
    for cidx, cam in enumerate(cself.CameraChain.camList):
        if len(cam.allReprojectionErrors)>0:
            e2 = np.array([ np.linalg.norm(rerr.error()) for reprojectionErrors in cam.allReprojectionErrors for rerr in reprojectionErrors])
            print >> dest, "Reprojection error (cam{0}) [px]:     mean {1}, median {2}, std: {3}".format(cidx, np.mean(e2), np.median(e2), np.std(e2) )
        else:
            print >> dest, "Reprojection error (cam{0}) [px]:     no corners".format(cidx)
    
    for iidx, imu in enumerate(cself.ImuList):
        # Gyro errors
        e2 = np.array([ np.linalg.norm(e.error()) for e in imu.gyroErrors ])
        print >> dest, "Gyroscope error (imu{0}) [rad/s]:     mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2))
        # Accelerometer errors
        e2 = np.array([ np.linalg.norm(e.error()) for e in imu.accelErrors ])
        print >> dest, "Accelerometer error (imu{0}) [m/s^2]: mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2))

def printGravity(cself):
    print
    print "Gravity vector: (in target coordinates): [m/s^2]"
    print cself.gravityDv.toEuclidean()

def printResults(cself, withCov=False):
    nCams = len(cself.CameraChain.camList)
    for camNr in range(0,nCams):
        T_cam_b = cself.CameraChain.getResultTrafoImuToCam(camNr)

        print
        print "Transformation T_cam{0}_imu0 (imu0 to cam{0}, T_ci): ".format(camNr)
        if withCov and camNr==0:
            print "\t quaternion: ", T_cam_b.q(), " +- ", cself.std_trafo_ic[0:3]
            print "\t translation: ", T_cam_b.t(), " +- ", cself.std_trafo_ic[3:]
        print T_cam_b.T()
        
        if not cself.noTimeCalibration:
            print
            print "cam{0} to imu0 time: [s] (t_imu = t_cam + shift)".format(camNr)
            print cself.CameraChain.getResultTimeShift(camNr),
            
            if withCov:
                print " +- ", cself.std_times[camNr]
            else:
                print

    print
    for (imuNr, imu) in enumerate(cself.ImuList):
        print "IMU{0}:\n".format(imuNr), "----------------------------"
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
            
            print
            print "Baseline (cam{0} to cam{1}): [m] {2}".format(camNr, camNr+1, isFixed)
            print T.T()
            print baseline, "[m]"
    
   
def generateReport(cself, filename="report.pdf", showOnScreen=True):  
    figs = list()
    plotter = PlotCollection.PlotCollection("Calibration report")
    offset = 3010
    
    #Output calibration results in text form.
    sstream = StringIO.StringIO()
    printResultTxt(cself, sstream)
    
    text = [line for line in StringIO.StringIO(sstream.getvalue())]
    linesPerPage = 40
    
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

        printText = lambda t: ax.text(left, top, t, fontsize=8, \
                                     horizontalalignment='left', verticalalignment='top',\
                                     transform=ax.transAxes)
        
        if len(text) > linesPerPage:
            printText("".join(text[0:linesPerPage]))
            figs.append(fig)
            text = text[linesPerPage:]
        else:
            printText("".join(text[0:]))
            figs.append(fig)
            break
    
    #plot imu stuff (if we have imus)
    for iidx, imu in enumerate(cself.ImuList):
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

def saveResultTxt(cself, filename='cam_imu_result.txt'):
    f = open(filename, 'w')
    printResultTxt(cself, stream=f)

def printResultTxt(cself, stream=sys.stdout):
    
    print >> stream, "Calibration results"
    print >> stream, "==================="   
    printErrorStatistics(cself, stream)
  
    # Calibration results
    nCams = len(cself.CameraChain.camList)
    for camNr in range(0,nCams):
        T = cself.CameraChain.getResultTrafoImuToCam(camNr)
        print >> stream, ""
        print >> stream, "Transformation (cam{0}):".format(camNr)
        print >> stream, "-----------------------"
        print >> stream, "T_ci:  (imu0 to cam{0}): ".format(camNr)   
        print >> stream, T.T()
        print >> stream, ""
        print >> stream, "T_ic:  (cam{0} to imu0): ".format(camNr)   
        print >> stream, T.inverse().T()
    
        # Time
        print >> stream, ""
        print >> stream, "timeshift cam{0} to imu0: [s] (t_imu = t_cam + shift)".format(camNr)
        print >> stream, cself.CameraChain.getResultTimeShift(camNr)
        print >> stream, ""

    #print all baselines in the camera chain
    if nCams > 1:
        print >> stream, "Baselines:"
        print >> stream, "----------"

        for camNr in range(0,nCams-1):
            T, baseline = cself.CameraChain.getResultBaseline(camNr, camNr+1)
            print >> stream, "Baseline (cam{0} to cam{1}): ".format(camNr, camNr+1)
            print >> stream, T.T()
            print >> stream, "baseline norm: ", baseline,  "[m]"
            print >> stream, ""
    
    # Gravity
    print >> stream, ""
    print >> stream, "Gravity vector in target coords: [m/s^2]"
    print >> stream, cself.gravityDv.toEuclidean()
    
    print >> stream, ""
    print >> stream, ""
    print >> stream, "Calibration configuration"
    print >> stream, "========================="
    print >> stream, ""

    for camNr, cam in enumerate( cself.CameraChain.camList ):
        print >> stream, "cam{0}".format(camNr)
        print >> stream, "-----"
        cam.camConfig.printDetails(stream)
        cam.targetConfig.printDetails(stream)
        print >> stream, ""
    
	print >> stream, ""
    print >> stream, ""
    print >> stream, "IMU configuration"
    print >> stream, "================="
    print >> stream, ""
    for (imuNr, imu) in enumerate(cself.ImuList):
        print >> stream, "IMU{0}:\n".format(imuNr), "----------------------------"
        imu.getImuConfig().printDetails(stream)
        print >> stream, ""
